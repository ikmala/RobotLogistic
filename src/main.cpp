#include <Arduino.h>

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
// Kalau nanti ada yang kepanjangan serong, mainkan:
// - compFwd / compRev (scale RPM)
// - minFwd / minRev   (batas start PWM)
// - pid.kp / pid.ki / pid.kd untuk masing-masing motor
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

  // Dinamis sedikit tergantung target (semakin kencang, minPWM boleh naik dikit)
  float dyn = base + 0.25f * fabs(targetTick);
  dyn = constrain(dyn, base, 160.0f);

  // Map |cmd| (0..255) → [dyn .. 255]
  float out = dyn + (255.0f - dyn) * (fabs(cmd) / 255.0f);

  return (cmd > 0.0f) ? (int)out : -(int)out;
}

/* ===========================================================
   GLOBAL & PARAMETER TUNING
   =========================================================== */

// perintah PWM "mentah" (sebelum minPWM & kompensasi)
float motorCmd[MOTOR_COUNT] = {0};

// target tick/loop maksimum
const float MAX_TICK   = 50.0f;

// batas akselerasi PWM per loop
const float ACCEL_STEP = 8.0f;

// smoothing input joystick
float Xs = 0.0f, Ys = 0.0f, Rs = 0.0f;

// TRIM input X beda maju & mundur (supaya maju dan mundur sama2 lurus)
float TRIM_X_FWD =  0.005f;   // saat joystick Y ≥ 0 (maju)
float TRIM_X_REV = -0.005f;   // saat joystick Y <  0 (mundur)
float TRIM_R     =  0.0f;     // kalau robot suka muter saat maju mundur

// loop timing
unsigned long lastLoopMs = 0;

// Debug print
#define DEBUG_PRINT 1
uint16_t debugCounter = 0;

/* ===========================================================
   SETUP
   =========================================================== */
void setup() {
  Serial.begin(115200);

  // PPM input
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  // Encoder pins
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    lastEnc[i] = 0;
    velFilt[i] = 0;
  }

  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), enc2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), enc3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[3]), enc4ISR, CHANGE);

  // Motor PWM awal 0
  for (int i = 0; i < MOTOR_COUNT; i++) {
    driveMotor(MOTOR_PIN_L[i], MOTOR_PIN_R[i], 0);
  }

  Serial.println("Robot OMNI – FINAL PID + Auto-Trim READY");
}

/* ===========================================================
   MAIN LOOP
   =========================================================== */
void loop() {
  unsigned long nowMs = millis();
  float dt = (nowMs - lastLoopMs) / 1000.0f;
  if (dt < 0.005f) return;     // ~200 Hz
  lastLoopMs = nowMs;

  /* ---------------------------------------------------
     FAILSAFE PPM
     --------------------------------------------------- */
  bool ppmOk = true;
  unsigned long nowUs = micros();
  if (nowUs - lastPpmUpdate > 200000UL) {   // > 200 ms tanpa PPM
    ppmOk = false;
  }

  // Baca channel (PPM) – kalau invalid atau failsafe → netral
  int ch1 = ppmOk ? ppm[0] : 1500;
  int ch2 = ppmOk ? ppm[1] : 1500;
  int ch3 = ppmOk ? ppm[2] : 1000;  // gas = 0
  int ch4 = ppmOk ? ppm[3] : 1500;

  if (ch1 < 900 || ch1 > 2100) ch1 = 1500;
  if (ch2 < 900 || ch2 > 2100) ch2 = 1500;
  if (ch3 < 900 || ch3 > 2100) ch3 = 1000;
  if (ch4 < 900 || ch4 > 2100) ch4 = 1500;

  // Normalisasi joystick: -1 .. +1
  float Xr = (ch1 - 1500) / 500.0f;
  float Yr = (ch2 - 1500) / 500.0f;
  float Rr = (ch4 - 1500) / 500.0f;
  float gas = constrain((ch3 - 1000) / 1000.0f, 0.0f, 1.0f);

  // Throttle kecil → matikan saja (supaya diam benar2 diam)
  if (gas < 0.02f || !ppmOk) {
    gas = 0.0f;
    for (int i = 0; i < MOTOR_COUNT; i++) {
      motorCmd[i] = 0;
      motors[i].pid.integral = 0;
      motors[i].pid.prevErr  = 0;
      driveMotor(MOTOR_PIN_L[i], MOTOR_PIN_R[i], 0);
    }

#if DEBUG_PRINT
    debugCounter++;
    if (debugCounter >= 50) {   // print lebih jarang saat idle
      debugCounter = 0;
      Serial.println("ENC: 0,0,0,0");
    }
#endif

    return;
  }

  /* ---------------------------------------------------
     APLIKASI TRIM INPUT
     --------------------------------------------------- */
  float trimX = (Yr >= 0.0f) ? TRIM_X_FWD : TRIM_X_REV;

  float X = expo(deadzone(Xr - trimX));
  float Y = expo(deadzone(Yr));
  float R = expo(deadzone(Rr - TRIM_R));

  // Smooth sedikit
  Xs = smooth(Xs, X, 0.08f);
  Ys = smooth(Ys, Y, 0.08f);
  Rs = smooth(Rs, R, 0.08f);

  /* ---------------------------------------------------
     KINEMATIKA OMNI 4 RODA
     Index:
       0 = M1 (kiri depan)
       1 = M2 (kanan depan)
       2 = M3 (kiri belakang)
       3 = M4 (kanan belakang)
     --------------------------------------------------- */
  float mix[4];
  mix[0] = Ys - Xs - Rs;  // M1
  mix[1] = Ys + Xs + Rs;  // M2
  mix[2] = Ys - Xs + Rs;  // M3
  mix[3] = Ys + Xs - Rs;  // M4

  // Normalize kalau lebih dari 1
  float maxv = 0.0f;
  for (int i = 0; i < 4; i++) {
    if (fabs(mix[i]) > maxv) maxv = fabs(mix[i]);
  }
  if (maxv > 1.0f) {
    for (int i = 0; i < 4; i++) mix[i] /= maxv;
  }

  // Target tick per period (loop) untuk tiap motor
  float tgt[4];
  for (int i = 0; i < 4; i++) {
    tgt[i] = mix[i] * MAX_TICK * gas;
  }

  /* ---------------------------------------------------
     ENCODER VELOCITY PER LOOP
     --------------------------------------------------- */
  long snap[4];
  for (int i = 0; i < 4; i++) {
    snap[i] = encCount[i];
    float v = (float)(snap[i] - lastEnc[i]);
    lastEnc[i] = snap[i];
    velFilt[i] = lowPass(velFilt[i], v, 0.40f);  // 0.4 = lumayan responsif
  }

  /* ---------------------------------------------------
     PID + FEEDFORWARD + ACCEL LIMIT + KOMPENSASI
     --------------------------------------------------- */
  float pwmLimit = 255.0f * gas;

  for (int i = 0; i < 4; i++) {
    // FEEDFORWARD (direct proportional ke target)
    const float FF_GAIN = 2.3f;
    float ff = tgt[i] * FF_GAIN;

    float corr = pidUpdate(motors[i].pid, tgt[i], velFilt[i], dt);

    float desired = ff + corr;

    // Acceleration limit
    float delta = desired - motorCmd[i];
    delta = constrain(delta, -ACCEL_STEP, ACCEL_STEP);
    motorCmd[i] += delta;

    // Kompensasi berbeda maju/mundur
    float comp = (motorCmd[i] >= 0.0f) ? motors[i].compFwd : motors[i].compRev;
    float adj = motorCmd[i] * comp;

    // Apply min PWM & clamp ke gas
    int pwm = applyMinPwm(adj, motors[i], tgt[i]);
    pwm = constrain(pwm, -(int)pwmLimit, (int)pwmLimit);

    driveMotor(MOTOR_PIN_L[i], MOTOR_PIN_R[i], pwm);
  }

  /* ---------------------------------------------------
     DEBUG PRINT (ENCODER)
     --------------------------------------------------- */
#if DEBUG_PRINT
  debugCounter++;
  if (debugCounter >= 10) {   // setiap ~10 loop ≈ 20 Hz
    debugCounter = 0;
    Serial.print("ENC: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(velFilt[i], 2);
      if (i < 3) Serial.print(",");
    }
    Serial.println();
  }
#endif
}
