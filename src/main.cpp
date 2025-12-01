#include <Arduino.h>

// ========== micro-ROS ==========
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

/* ===========================================================
   PPM INPUT
   =========================================================== */
#if defined(ARDUINO_ARCH_ESP32)
constexpr uint8_t PPM_PIN = 34;   // sesuaikan dengan pin PPM di ESP32-mu
#else
constexpr uint8_t PPM_PIN = 18;
#endif

constexpr size_t CHANNELS = 6;

volatile uint16_t ppm[CHANNELS];
volatile uint8_t  ppmIndex = 0;
volatile unsigned long lastMicrosPPM = 0;
volatile unsigned long lastPpmUpdate = 0;

void ppmISR() {
  unsigned long now = micros();
  unsigned long dur = now - lastMicrosPPM;
  lastMicrosPPM = now;

  if (dur > 3000) {
    ppmIndex = 0;
  } else if (ppmIndex < CHANNELS) {
    ppm[ppmIndex++] = dur;
  }

  lastPpmUpdate = now;
}

/* ===========================================================
   MOTOR ORIENTASI
   M1 = kiri depan   (PG45)
   M2 = kanan depan  (PG45)
   M3 = kiri belakang (PG36)
   M4 = kanan belakang (PG36)
   =========================================================== */

constexpr size_t MOTOR_COUNT = 4;

#if defined(ARDUINO_ARCH_ESP32)
const uint8_t MOTOR_PIN_L[MOTOR_COUNT] = {13, 14, 26, 27};
const uint8_t MOTOR_PIN_R[MOTOR_COUNT] = {25, 23, 19, 18};
const uint8_t ENC_A[MOTOR_COUNT]      = {32, 33, 4, 5};
const uint8_t ENC_B[MOTOR_COUNT]      = {16, 17, 21, 22};
#else
const uint8_t MOTOR_PIN_L[MOTOR_COUNT] = {5, 7, 9, 11};
const uint8_t MOTOR_PIN_R[MOTOR_COUNT] = {6, 8, 10, 12};
const uint8_t ENC_A[MOTOR_COUNT]       = {19, 20, 21, 3};
const uint8_t ENC_B[MOTOR_COUNT]       = {22, 23, 24, 25};
#endif

// arah encoder (+) = maju
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
   DRIVE MOTOR
   =========================================================== */
#if defined(ARDUINO_ARCH_ESP32)
uint8_t pwmChannelL[MOTOR_COUNT];
uint8_t pwmChannelR[MOTOR_COUNT];
constexpr uint32_t MOTOR_PWM_FREQ = 20000;
constexpr uint8_t MOTOR_PWM_RES_BITS = 8;

void setupMotorPWM() {
  uint8_t channel = 0;
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    pwmChannelL[i] = channel++;
    pwmChannelR[i] = channel++;
    ledcSetup(pwmChannelL[i], MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);
    ledcSetup(pwmChannelR[i], MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);
    ledcAttachPin(MOTOR_PIN_L[i], pwmChannelL[i]);
    ledcAttachPin(MOTOR_PIN_R[i], pwmChannelR[i]);
  }
}

inline void writeMotorPWM(uint8_t idx, bool leftSide, uint8_t value) {
  ledcWrite(leftSide ? pwmChannelL[idx] : pwmChannelR[idx], value);
}
#else
inline void setupMotorPWM() {}

inline void writeMotorPWM(uint8_t idx, bool leftSide, uint8_t value) {
  analogWrite(leftSide ? MOTOR_PIN_L[idx] : MOTOR_PIN_R[idx], value);
}
#endif

void driveMotor(uint8_t motorIndex, int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    writeMotorPWM(motorIndex, true, static_cast<uint8_t>(pwm));
    writeMotorPWM(motorIndex, false, 0);
  } else if (pwm < 0) {
    writeMotorPWM(motorIndex, true, 0);
    writeMotorPWM(motorIndex, false, static_cast<uint8_t>(-pwm));
  } else {
    writeMotorPWM(motorIndex, true, 0);
    writeMotorPWM(motorIndex, false, 0);
  }
}

/* ===========================================================
   FILTERS
   =========================================================== */
float smooth(float last, float cur, float k) { return last + (cur - last) * k; }

float expo(float x, float e = 0.35f) {
  return (1.0f - e) * x + e * x * x * x;
}

float deadzone(float x) {
  const float d = 0.06f;
  if (fabsf(x) < d) return 0.0f;
  return (x > 0) ? (x - d) / (1.0f - d)
                 : (x + d) / (1.0f - d);
}

/* ===========================================================
   PID & TUNING
   =========================================================== */
struct PID {
  float kp, ki, kd;
  float integral;
  float prevErr;
};

struct MotorTune {
  int   minFwd;
  int   minRev;
  float compFwd;
  float compRev;
  PID   pid;
};

// Tuning hasil dari test encoder
MotorTune motors[MOTOR_COUNT] = {
  // M1 = PG45 kiri depan
  {60, 65, 1.15f, 1.15f, {3.5f, 0.03f, 0.20f, 0, 0}},  

  // M2 = PG45 kanan depan
  {60, 65, 1.50f, 1.50f, {4.1f, 0.04f, 0.22f, 0, 0}}, 

  // M3 = PG36 kiri belakang
  {50, 55, 0.72f, 0.72f, {2.2f, 0.045f, 0.20f, 0, 0}},  

  // M4 = PG36 kanan belakang
  {50, 55, 0.76f, 0.76f, {2.3f, 0.045f, 0.20f, 0, 0}}  
};

float pidUpdate(PID &pid, float target, float meas, float dt) {
  float err = target - meas;
  pid.integral += err * dt;
  pid.integral = constrain(pid.integral, -150.0f, 150.0f);

  float deriv = (err - pid.prevErr) / dt;
  pid.prevErr = err;

  return pid.kp * err + pid.ki * pid.integral + pid.kd * deriv;
}

/* ===========================================================
   ENCODER VELOCITY
   =========================================================== */
float velFilt[4] = {0};
long  lastEnc[4] = {0};

float lowPass(float prev, float cur, float k) {
  return prev * (1.0f - k) + cur * k;
}

/* ===========================================================
   micro-ROS: node + publisher kecepatan roda
   =========================================================== */

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

rcl_publisher_t pub_m1, pub_m2, pub_m3, pub_m4;
std_msgs__msg__Float32 msg_m1, msg_m2, msg_m3, msg_m4;

bool microros_ok = false;

void microros_timer_callback(rcl_timer_t *t, int64_t last_call_time) {
  (void)t;
  (void)last_call_time;
  if (!microros_ok) return;

  // publish velFilt (tick per loop yang sudah difilter)
  msg_m1.data = velFilt[0];
  msg_m2.data = velFilt[1];
  msg_m3.data = velFilt[2];
  msg_m4.data = velFilt[3];

  if (rcl_publish(&pub_m1, &msg_m1, NULL) != RCL_RET_OK ||
      rcl_publish(&pub_m2, &msg_m2, NULL) != RCL_RET_OK ||
      rcl_publish(&pub_m3, &msg_m3, NULL) != RCL_RET_OK ||
      rcl_publish(&pub_m4, &msg_m4, NULL) != RCL_RET_OK) {
    microros_ok = false;
    Serial.println("micro-ROS publish error, stop sending.");
  }
}

void setup_microros() {
  // Transport pakai USB bawaan (Serial)
  set_microros_transports();

  allocator = rcl_get_default_allocator();

  // Cek agent micro-ROS
  const uint8_t attempts = 10;
  const uint32_t timeout_ms = 500;
  bool agent_ok = false;
  for (uint8_t i = 0; i < attempts; i++) {
    if (rmw_uros_ping_agent(timeout_ms, 1) == RMW_RET_OK) {
      agent_ok = true;
      break;
    }
    delay(500);
  }

  if (!agent_ok) {
    Serial.println("micro-ROS agent TIDAK TERDETEKSI, jalan tanpa ROS.");
    return;
  }

  rcl_ret_t rc;

  rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK) {
    Serial.println("rclc_support_init FAILED");
    return;
  }

  rc = rclc_node_init_default(&node, "esp32_omni_base", "", &support);
  if (rc != RCL_RET_OK) {
    Serial.println("node_init FAILED");
    return;
  }

  rc = rclc_publisher_init_default(
      &pub_m1, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "wheel1/vel");
  if (rc != RCL_RET_OK) { Serial.println("pub_m1 FAILED"); return; }

  rc = rclc_publisher_init_default(
      &pub_m2, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "wheel2/vel");
  if (rc != RCL_RET_OK) { Serial.println("pub_m2 FAILED"); return; }

  rc = rclc_publisher_init_default(
      &pub_m3, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "wheel3/vel");
  if (rc != RCL_RET_OK) { Serial.println("pub_m3 FAILED"); return; }

  rc = rclc_publisher_init_default(
      &pub_m4, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "wheel4/vel");
  if (rc != RCL_RET_OK) { Serial.println("pub_m4 FAILED"); return; }

  rc = rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(50),   // 50 ms → 20 Hz publish
      microros_timer_callback);
  if (rc != RCL_RET_OK) {
    Serial.println("timer_init FAILED");
    return;
  }

  rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (rc != RCL_RET_OK) {
    Serial.println("executor_init FAILED");
    return;
  }

  rc = rclc_executor_add_timer(&executor, &timer);
  if (rc != RCL_RET_OK) {
    Serial.println("add_timer FAILED");
    return;
  }

  microros_ok = true;
  Serial.println("micro-ROS INITIALIZED (wheel vel publishers ready).");
}

/* ===========================================================
   OPEN LOOP BASE + PID + FEEDFORWARD
   =========================================================== */
float motorCmd[4] = {0};

const float MAX_TICK   = 50.0f;
const float ACCEL_STEP = 8.0f;

/* ===========================================================
   TRIM & STATE
   =========================================================== */
float Xs = 0, Ys = 0, Rs = 0;

float TRIM_X_FWD =  0.005f;
float TRIM_X_REV = -0.005f;
float TRIM_R     =  0.0f;

unsigned long lastLoopMs = 0;

#define DEBUG_PRINT 1
uint16_t debugCounter = 0;

/* ===========================================================
   SETUP
   =========================================================== */
void setup() {
  Serial.begin(115200);
  delay(2000);  // beri waktu USB nyambung

  // PPM
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  // Motor pins
  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_PIN_L[i], OUTPUT);
    pinMode(MOTOR_PIN_R[i], OUTPUT);
  }
  setupMotorPWM();
  for (int i = 0; i < 4; i++) {
    driveMotor(i, 0);
  }

  // Encoder pins
  for (int i = 0; i < 4; i++) {
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), enc2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), enc3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[3]), enc4ISR, CHANGE);

  Serial.println("Robot OMNI PG45 depan + PG36 belakang - ENCODER + PID READY");

  // Init micro-ROS (non fatal kalau agent belum nyala)
  setup_microros();
}

/* ===========================================================
   LOOP
   =========================================================== */
void loop() {

  // micro-ROS executor jalan setiap loop (ringan)
  if (microros_ok) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  }

  /* LOOP RATE ~200 Hz */
  unsigned long nowMs = millis();
  float dt = (nowMs - lastLoopMs) / 1000.0f;
  if (dt < 0.005f) return;
  if (dt > 0.02f) dt = 0.005f; // clamp agar PID stabil
  lastLoopMs = nowMs;

  /* FAILSAFE PPM */
  bool ppmOk = (micros() - lastPpmUpdate) < 200000UL;

  int ch1 = ppmOk ? ppm[0] : 1500;
  int ch2 = ppmOk ? ppm[1] : 1500;
  int ch3 = ppmOk ? ppm[2] : 1000;
  int ch4 = ppmOk ? ppm[3] : 1500;

  if (ch1 < 900 || ch1 > 2100) ch1 = 1500;
  if (ch2 < 900 || ch2 > 2100) ch2 = 1500;
  if (ch3 < 900 || ch3 > 2100) ch3 = 1000;
  if (ch4 < 900 || ch4 > 2100) ch4 = 1500;

  /* NORMALISASI INPUT REMOTE */
  float Xr = (ch1 - 1500) / 500.0f;
  float Yr = (ch2 - 1500) / 500.0f;
  float Rr = (ch4 - 1500) / 500.0f;
  float gas = constrain((ch3 - 1000) / 1000.0f, 0.0f, 1.0f);

  // Throttle kecil / PPM hilang → full stop + reset PID
  if (gas < 0.02f || !ppmOk) {
    for (int i = 0; i < 4; i++) {
      motorCmd[i] = 0;
      motors[i].pid.integral = 0;
      motors[i].pid.prevErr  = 0;
      driveMotor(i, 0);
    }
#if DEBUG_PRINT
    debugCounter++;
    if (debugCounter >= 50) {
      debugCounter = 0;
      Serial.println("VEL:0,0,0,0");
    }
#endif
    return;
  }

  /* TRIM + EXPO + SMOOTH INPUT */
  float trimX = (Yr >= 0.0f) ? TRIM_X_FWD : TRIM_X_REV;

  float X = expo(deadzone(Xr - trimX));
  float Y = expo(deadzone(Yr));
  float R = expo(deadzone(Rr - TRIM_R));

  Xs = smooth(Xs, X, 0.05f);
  Ys = smooth(Ys, Y, 0.05f);
  Rs = smooth(Rs, R, 0.05f);

  // Kalau stick sangat kecil → matikan PID & motor (anti jitter)
  if (fabsf(Xs) < 0.05f && fabsf(Ys) < 0.05f && fabsf(Rs) < 0.05f) {
    for (int i = 0; i < 4; i++) {
      motorCmd[i] = 0;
      motors[i].pid.integral = 0;
      motors[i].pid.prevErr  = 0;
      driveMotor(i, 0);
    }
#if DEBUG_PRINT
    debugCounter++;
    if (debugCounter >= 70) {
      debugCounter = 0;
      Serial.println("VEL:0,0,0,0");
    }
#endif
    return;
  }

  /* BACA ENCODER → KECEPATAN PER LOOP */
  long snap[4];
  for (int i = 0; i < 4; i++) {
    snap[i] = encCount[i];
    float v = (float)(snap[i] - lastEnc[i]);
    lastEnc[i] = snap[i];

    velFilt[i] = lowPass(velFilt[i], v, 0.40f);

    // DEADZONE ENCODER (anti noise kecil)
    if (fabsf(velFilt[i]) < 0.8f) velFilt[i] = 0.0f;
  }

  /* OMNI KINEMATICS (X=strafe, Y=maju, R=rotasi) */
  float mix[4];
  mix[0] = Ys - Xs - Rs;  // M1 kiri depan
  mix[1] = Ys + Xs + Rs;  // M2 kanan depan
  mix[2] = Ys - Xs + Rs;  // M3 kiri belakang
  mix[3] = Ys + Xs - Rs;  // M4 kanan belakang

  float maxv = 0.0f;
  for (int i = 0; i < 4; i++) {
    if (fabsf(mix[i]) > maxv) maxv = fabsf(mix[i]);
  }
  if (maxv > 1.0f) {
    for (int i = 0; i < 4; i++) mix[i] /= maxv;
  }

  /* TARGET VELOCITY (TICK/LOOP) */
  float tgt[4];
  for (int i = 0; i < 4; i++) {
    tgt[i] = mix[i] * MAX_TICK * gas;
  }

  /* PID + FEEDFORWARD + KOMPENSASI MOTOR */
  const float FF_GAIN = 1.7f;  // lebih halus dari 2.3

  float pwmLimit = 255.0f * gas;

  for (int i = 0; i < 4; i++) {

    float ff   = tgt[i] * FF_GAIN;
    float corr = pidUpdate(motors[i].pid, tgt[i], velFilt[i], dt);
    float desired = ff + corr;

    // ACCELERATION LIMIT
    float delta = desired - motorCmd[i];
    if (delta >  ACCEL_STEP) delta =  ACCEL_STEP;
    if (delta < -ACCEL_STEP) delta = -ACCEL_STEP;
    motorCmd[i] += delta;

    // KOMPENSASI PG45 / PG36
    float adj =
      (motorCmd[i] >= 0.0f)
      ? motorCmd[i] * motors[i].compFwd
      : motorCmd[i] * motors[i].compRev;

    // APPLY MIN PWM
    int pwm;
    if (adj == 0.0f || tgt[i] == 0.0f) {
      pwm = 0;
    } else {
      bool fwd = (adj > 0.0f);
      int base = fwd ? motors[i].minFwd : motors[i].minRev;

      float dyn = base + 0.25f * fabsf(tgt[i]);
      if (dyn > 160.0f) dyn = 160.0f;

      float out = dyn + (255.0f - dyn) * (fabsf(adj) / 255.0f);
      pwm = fwd ? (int)out : -(int)out;
    }

    // BATAS SESUAI GAS
    if (pwm >  (int)pwmLimit) pwm =  (int)pwmLimit;
    if (pwm < -(int)pwmLimit) pwm = -(int)pwmLimit;

    driveMotor(i, pwm);
  }

  /* DEBUG PRINT */
#if DEBUG_PRINT
  debugCounter++;
  if (debugCounter >= 15) {   // ~13 Hz
    debugCounter = 0;
    Serial.print("VEL:");
    for (int i = 0; i < 4; i++) {
      Serial.print(velFilt[i], 1);
      if (i < 3) Serial.print(",");
    }
    Serial.println();
  }
#endif
}
