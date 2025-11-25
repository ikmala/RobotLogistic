#include <Arduino.h>
#include <ArduinoJson.h>

/* ===========================================================
   PPM INPUT (SINYAL REMOTE)
   =========================================================== */
#define PPM_PIN    18
#define CHANNELS   6

volatile uint16_t ppm[CHANNELS];
volatile uint8_t  ppmIndex = 0;
volatile unsigned long lastMicrosPPM = 0;
volatile unsigned long lastPpmUpdate = 0;   // untuk failsafe

void ppmISR() {
  unsigned long now = micros();
  unsigned long dur = now - lastMicrosPPM;
  lastMicrosPPM = now;

  if (dur > 3000) {
    ppmIndex = 0;
  } else if (ppmIndex < CHANNELS) {
    ppm[ppmIndex++] = dur;
  }

  lastPpmUpdate = now; // update waktu terakhir PPM valid
}

/* ===========================================================
   DEFINISI MOTOR & ENCODER
   Orientasi:
     M1 = kiri depan
     M2 = kanan depan
     M3 = kiri belakang
     M4 = kanan belakang
   =========================================================== */

constexpr size_t MOTOR_COUNT = 4;

// Pin driver BTS7960 (EN_L, EN_R) per motor
const uint8_t MOTOR_PIN_L[MOTOR_COUNT] = {A0, A9, A12, A7};
const uint8_t MOTOR_PIN_R[MOTOR_COUNT] = {A1, A11, A4, A6};

// Encoder pin (channel A & B)
const uint8_t ENC_A[MOTOR_COUNT] = {19, 20, 21, 3};
const uint8_t ENC_B[MOTOR_COUNT] = {22, 23, 24, 25};

// Arah encoder khusus tiap motor (supaya + = maju)
const int8_t ENC_DIR[MOTOR_COUNT] = {+1, -1, +1, -1};

volatile long encCount[MOTOR_COUNT] = {0};

void updateEncoder(uint8_t idx) {
  bool same = (digitalRead(ENC_A[idx]) == digitalRead(ENC_B[idx]));
  encCount[idx] += same ? ENC_DIR[idx] : -ENC_DIR[idx];
}

void enc1ISR() { updateEncoder(0); }
void enc2ISR() { updateEncoder(1); }
void enc3ISR() { updateEncoder(2); }
void enc4ISR() { updateEncoder(3); }

/* ===========================================================
   FUNGSI DRIVE MOTOR
   =========================================================== */
void driveMotor(uint8_t pinL, uint8_t pinR, int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(pinL, pwm);
    analogWrite(pinR, 0);
  } else if (pwm < 0) {
    analogWrite(pinL, 0);
    analogWrite(pinR, -pwm);
  } else {
    analogWrite(pinL, 0);
    analogWrite(pinR, 0);
  }
}

/* ===========================================================
   FILTER INPUT (SMOOTH, EXPO, DEADZONE)
   =========================================================== */
float smooth(float last, float cur, float k) {
  return last + (cur - last) * k;
}

float expo(float x, float e = 0.35f) {
  return (1.0f - e) * x + e * x * x * x;
}

float deadzone(float x) {
  const float d = 0.06f;
  if (fabs(x) < d) return 0.0f;
  return (x > 0) ? (x - d) / (1.0f - d)
                 : (x + d) / (1.0f - d);
}

/* ===========================================================
   PID KECEPATAN (PD + I kecil)
   =========================================================== */
struct PID {
  float kp, ki, kd;
  float integral;
  float prevErr;
};

struct MotorTune {
  int   minFwd;    // min PWM saat maju
  int   minRev;    // min PWM saat mundur
  float compFwd;   // kompensasi PWM maju
  float compRev;   // kompensasi PWM mundur
  PID   pid;
};

// TUNING PER MOTOR
MotorTune motors[MOTOR_COUNT] = {
  //  minFwd, minRev, compFwd, compRev,     {kp,   ki,   kd,   integral, prevErr}
  {70,  80,   1.00f,  1.00f,  {2.6f, 0.03f, 0.25f, 0.0f, 0.0f}},  // M1 - kiri depan
  {70,  80,   1.00f,  1.00f,  {2.6f, 0.03f, 0.25f, 0.0f, 0.0f}},  // M2 - kanan depan
  {70,  80,   1.00f,  1.00f,  {2.6f, 0.03f, 0.25f, 0.0f, 0.0f}},  // M3 - kiri belakang
  {70,  80,   1.00f,  1.00f,  {2.6f, 0.03f, 0.25f, 0.0f, 0.0f}}   // M4 - kanan belakang
};

float pidUpdate(PID &pid, float target, float meas, float dt) {
  float err = target - meas;

  pid.integral += err * dt;
  pid.integral = constrain(pid.integral, -150.0f, +150.0f);

  float deriv = (err - pid.prevErr) / dt;
  pid.prevErr = err;

  return pid.kp * err + pid.ki * pid.integral + pid.kd * deriv;
}

/* ===========================================================
   FILTER ENCODER (LOW PASS)
   =========================================================== */
float velFilt[MOTOR_COUNT] = {0};
long  lastEnc[MOTOR_COUNT] = {0};

float lowPass(float prev, float cur, float k) {
  return prev * (1.0f - k) + cur * k;
}

/* ===========================================================
   APLIKASI MIN PWM & KOMPENSASI
   =========================================================== */
int applyMinPwm(float cmd, const MotorTune &m, float targetTick) {
  if (cmd == 0.0f || targetTick == 0.0f) {
    return 0;
  }

  bool fwd = (cmd > 0.0f);
  int base = fwd ? m.minFwd : m.minRev;

  float dyn = base + 0.25f * fabs(targetTick);
  dyn = constrain(dyn, base, 160.0f);

  float out = dyn + (255.0f - dyn) * (fabs(cmd) / 255.0f);

  return (cmd > 0.0f) ? (int)out : -(int)out;
}

/* ===========================================================
   GLOBAL & PARAMETER TUNING
   =========================================================== */

float motorCmd[MOTOR_COUNT] = {0};
const float MAX_TICK   = 50.0f;
const float ACCEL_STEP = 8.0f;
float Xs = 0.0f, Ys = 0.0f, Rs = 0.0f;

float TRIM_X_FWD =  0.005f;   // saat joystick Y ≥ 0 (maju)
float TRIM_X_REV = -0.005f;   // saat joystick Y <  0 (mundur)
float TRIM_R     =  0.0f;     // kalau robot suka muter saat maju mundur

unsigned long lastLoopMs = 0;

#define DEBUG_PRINT 0
uint16_t debugCounter = 0;

/* ===========================================================
   SERIAL BRIDGE STATE
   =========================================================== */
const unsigned long SERIAL_CMD_TIMEOUT_MS = 300;
const unsigned long TELEMETRY_PERIOD_MS   = 20;
const float AUTO_CMD_THRESHOLD = 0.05f;
const float MANUAL_GAS_THRESHOLD = 0.05f;

char serialLine[160];
size_t serialPos = 0;
StaticJsonDocument<200> rxJson;

float autoX = 0.0f, autoY = 0.0f, autoR = 0.0f;
unsigned long lastSerialCmdMs = 0;
unsigned long lastTelemetryMs = 0;
bool autonomousMode = false;

void applySerialCommand(float x, float y, float r) {
  autoX = constrain(x, -1.0f, 1.0f);
  autoY = constrain(y, -1.0f, 1.0f);
  autoR = constrain(r, -1.0f, 1.0f);
  lastSerialCmdMs = millis();
}

void processJsonCommand() {
  float cmdX = 0.0f;
  float cmdY = 0.0f;
  float cmdR = 0.0f;

  if (rxJson.containsKey("cmd_x")) cmdX = rxJson["cmd_x"];
  else if (rxJson.containsKey("cmd_lat")) cmdX = rxJson["cmd_lat"];
  else if (rxJson.containsKey("cmd_y")) cmdX = rxJson["cmd_y"];

  if (rxJson.containsKey("cmd_y")) cmdY = rxJson["cmd_y"];
  else if (rxJson.containsKey("cmd_lin")) cmdY = rxJson["cmd_lin"];

  if (rxJson.containsKey("cmd_r")) cmdR = rxJson["cmd_r"];
  else if (rxJson.containsKey("cmd_ang")) cmdR = rxJson["cmd_ang"];
  else if (rxJson.containsKey("cmd_w")) cmdR = rxJson["cmd_w"];

  applySerialCommand(cmdX, cmdY, cmdR);
}

void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (serialPos > 0) {
        serialLine[serialPos] = '\0';
        DeserializationError err = deserializeJson(rxJson, serialLine);
        if (!err) {
          processJsonCommand();
        }
      }
      serialPos = 0;
    } else {
      if (serialPos < sizeof(serialLine) - 1) {
        serialLine[serialPos++] = c;
      } else {
        serialPos = 0; // overflow, drop line
      }
    }
  }
}

void sendTelemetry(unsigned long nowMs, float gas, bool ppmOk) {
  if (nowMs - lastTelemetryMs < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = nowMs;

  long encSnap[MOTOR_COUNT];
  noInterrupts();
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    encSnap[i] = encCount[i];
  }
  interrupts();

  StaticJsonDocument<320> tx;
  tx["mode"] = autonomousMode ? "auto" : "manual";
  tx["gas"] = gas;
  tx["ppm"] = ppmOk ? 1 : 0;
  tx["xs"] = Xs;
  tx["ys"] = Ys;
  tx["rs"] = Rs;
  JsonArray encArr = tx.createNestedArray("enc");
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    encArr.add(encSnap[i]);
  }
  JsonArray velArr = tx.createNestedArray("vel");
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    velArr.add(velFilt[i]);
  }
  serializeJson(tx, Serial);
  Serial.println();
}

void stopAllMotors() {
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    motorCmd[i] = 0;
    motors[i].pid.integral = 0;
    motors[i].pid.prevErr  = 0;
    driveMotor(MOTOR_PIN_L[i], MOTOR_PIN_R[i], 0);
  }
}

/* ===========================================================
   SETUP
   =========================================================== */
void setup() {
  Serial.begin(115200);

  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    lastEnc[i] = 0;
    velFilt[i] = 0;
  }

  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), enc2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), enc3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[3]), enc4ISR, CHANGE);

  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    driveMotor(MOTOR_PIN_L[i], MOTOR_PIN_R[i], 0);
  }
}

/* ===========================================================
   MAIN LOOP
   =========================================================== */
void loop() {
  unsigned long nowMs = millis();
  float dt = (nowMs - lastLoopMs) / 1000.0f;
  if (dt < 0.005f) {
    handleSerialInput();
    sendTelemetry(nowMs, 0.0f, true);
    return;
  }
  lastLoopMs = nowMs;

  handleSerialInput();

  bool serialFresh = (nowMs - lastSerialCmdMs) <= SERIAL_CMD_TIMEOUT_MS;
  float autoMag = max(max(fabs(autoX), fabs(autoY)), fabs(autoR));
  bool autoCmdActive = serialFresh && (autoMag > AUTO_CMD_THRESHOLD);

  bool ppmOk = true;
  unsigned long nowUs = micros();
  if (nowUs - lastPpmUpdate > 200000UL) {
    ppmOk = false;
  }

  int ch1 = ppmOk ? ppm[0] : 1500;
  int ch2 = ppmOk ? ppm[1] : 1500;
  int ch3 = ppmOk ? ppm[2] : 1000;
  int ch4 = ppmOk ? ppm[3] : 1500;

  if (ch1 < 900 || ch1 > 2100) ch1 = 1500;
  if (ch2 < 900 || ch2 > 2100) ch2 = 1500;
  if (ch3 < 900 || ch3 > 2100) ch3 = 1000;
  if (ch4 < 900 || ch4 > 2100) ch4 = 1500;

  float Xr = (ch1 - 1500) / 500.0f;
  float Yr = (ch2 - 1500) / 500.0f;
  float Rr = (ch4 - 1500) / 500.0f;
  float gas = constrain((ch3 - 1000) / 1000.0f, 0.0f, 1.0f);

  bool manualActive = (gas >= MANUAL_GAS_THRESHOLD) && ppmOk;
  if (manualActive) {
    autonomousMode = false;
  } else if (autoCmdActive) {
    autonomousMode = true;
  } else if (!serialFresh) {
    autonomousMode = false;
  }

  if ((gas < 0.02f || !ppmOk) && !autonomousMode) {
    stopAllMotors();
    sendTelemetry(nowMs, gas, ppmOk);
    return;
  }

  float trimX = (Yr >= 0.0f) ? TRIM_X_FWD : TRIM_X_REV;

  float manualX = expo(deadzone(Xr - trimX));
  float manualY = expo(deadzone(Yr));
  float manualR = expo(deadzone(Rr - TRIM_R));

  if (autonomousMode) {
    Xs = smooth(Xs, constrain(autoX, -1.0f, 1.0f), 0.10f);
    Ys = smooth(Ys, constrain(autoY, -1.0f, 1.0f), 0.10f);
    Rs = smooth(Rs, constrain(autoR, -1.0f, 1.0f), 0.10f);
  } else {
    Xs = smooth(Xs, manualX, 0.08f);
    Ys = smooth(Ys, manualY, 0.08f);
    Rs = smooth(Rs, manualR, 0.08f);
  }

  float mix[4];
  mix[0] = Ys - Xs - Rs;  // M1
  mix[1] = Ys + Xs + Rs;  // M2
  mix[2] = Ys - Xs + Rs;  // M3
  mix[3] = Ys + Xs - Rs;  // M4

  float maxv = 0.0f;
  for (int i = 0; i < 4; i++) {
    if (fabs(mix[i]) > maxv) maxv = fabs(mix[i]);
  }
  if (maxv > 1.0f) {
    for (int i = 0; i < 4; i++) mix[i] /= maxv;
  }

  float effectiveGas = autonomousMode ? 1.0f : gas;

  float tgt[4];
  for (int i = 0; i < 4; i++) {
    tgt[i] = mix[i] * MAX_TICK * effectiveGas;
  }

  long snap[4];
  for (int i = 0; i < 4; i++) {
    snap[i] = encCount[i];
    float v = (float)(snap[i] - lastEnc[i]);
    lastEnc[i] = snap[i];
    velFilt[i] = lowPass(velFilt[i], v, 0.40f);
  }

  float pwmLimit = 255.0f * effectiveGas;

  for (int i = 0; i < 4; i++) {
    const float FF_GAIN = 2.3f;
    float ff = tgt[i] * FF_GAIN;

    float corr = pidUpdate(motors[i].pid, tgt[i], velFilt[i], dt);

    float desired = ff + corr;

    float delta = desired - motorCmd[i];
    delta = constrain(delta, -ACCEL_STEP, ACCEL_STEP);
    motorCmd[i] += delta;

    float comp = (motorCmd[i] >= 0.0f) ? motors[i].compFwd : motors[i].compRev;
    float adj = motorCmd[i] * comp;

    int pwm = applyMinPwm(adj, motors[i], tgt[i]);
    pwm = constrain(pwm, -(int)pwmLimit, (int)pwmLimit);

    driveMotor(MOTOR_PIN_L[i], MOTOR_PIN_R[i], pwm);
  }

#if DEBUG_PRINT
  debugCounter++;
  if (debugCounter >= 10) {
    debugCounter = 0;
    Serial.print("ENC: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(velFilt[i], 2);
      if (i < 3) Serial.print(",");
    }
    Serial.println();
  }
#endif

  sendTelemetry(nowMs, gas, ppmOk);
}
