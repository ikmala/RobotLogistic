#include <Arduino.h>
#define PPM_PIN 18
#define CHANNELS 6
constexpr size_t MOTOR_COUNT = 4;

volatile uint16_t ppm[CHANNELS];
volatile uint8_t ppmIndex = 0;
volatile unsigned long lastMicros = 0;

void ppmISR(){
    unsigned long now = micros();
    unsigned long dur = now - lastMicros;
    lastMicros = now;

    if(dur > 3000) ppmIndex = 0;
    else if(ppmIndex < CHANNELS) ppm[ppmIndex++] = dur;
}

// ==================================================
// MOTOR DRIVER PINS (urut: depan kiri, depan kanan, belakang kiri, belakang kanan)
// ==================================================
const uint8_t MOTOR_PIN_L[MOTOR_COUNT] = {A0, A9, A4, A6};
const uint8_t MOTOR_PIN_R[MOTOR_COUNT] = {A1, A11, A5, A7};

// ==================================================
// ENCODER PINS DAN STATE
// ==================================================
const uint8_t ENC_PIN_A[MOTOR_COUNT] = {19, 20, 21, 3};
const uint8_t ENC_PIN_B[MOTOR_COUNT] = {22, 23, 24, 25};
const int8_t ENC_DIR[MOTOR_COUNT]     = {+1, -1, +1, -1};

volatile long encoderCounts[MOTOR_COUNT] = {0};

void updateEncoder(uint8_t idx){
    bool same = digitalRead(ENC_PIN_A[idx]) == digitalRead(ENC_PIN_B[idx]);
    encoderCounts[idx] += same ? ENC_DIR[idx] : -ENC_DIR[idx];
}

void enc1ISR(){ updateEncoder(0); }
void enc2ISR(){ updateEncoder(1); }
void enc3ISR(){ updateEncoder(2); }
void enc4ISR(){ updateEncoder(3); }

// ==================================================
// MOTOR DRIVE
// ==================================================
void driveMotor(int L, int R, int speed){
    speed = constrain(speed, -255, 255);
    if(speed>0){ analogWrite(L, speed); analogWrite(R, 0); }
    else if(speed<0){ analogWrite(L, 0); analogWrite(R, -speed); }
    else{ analogWrite(L,0); analogWrite(R,0); }
}

// ==================================================
// SMOOTH / EXPO / DEADZONE
// ==================================================
float smooth(float last,float cur,float k){
    return last + (cur-last)*k;
}

float expo(float v, float e=0.35){
    return (1-e)*v + e*v*v*v;
}

float deadzone(float v){
    float d=0.06;        // deadzone agak dibesarkan
    if(abs(v)<d) return 0;
    return (v>0) ? (v-d)/(1-d) : (v+d)/(1-d);
}

// ==================================================
// P-ONLY "PID" STRUCT (kalau nanti mau dipakai)
// ==================================================
struct PID {
  float kp, ki, kd;
  float integral;
  float prevError;
};

struct MotorTuning {
    int minPwm;
    float compensation;
    PID pid;
};

MotorTuning motorTune[MOTOR_COUNT] = {
    {70, 0.97f, {2.0f, 0.15f, 0.0f, 0.0f, 0.0f}}, // Motor 1 (depan kiri)
    {60, 1.00f, {2.0f, 0.15f, 0.0f, 0.0f, 0.0f}}, // Motor 2 (depan kanan)
    {70, 0.96f, {2.0f, 0.15f, 0.0f, 0.0f, 0.0f}}, // Motor 3 (belakang kiri)
    {60, 1.00f, {2.0f, 0.15f, 0.0f, 0.0f, 0.0f}}  // Motor 4 (belakang kanan)
};

const float DT = 0.005f;   // 5 ms

// ==================================================
// FEEDFORWARD + BATAS PERUBAHAN PWM
// ==================================================
const float FF_GAIN    = 3.0f;   // Feedforward → target_tick -> PWM
const float ACCEL_STEP = 8.0f;   // Batas perubahan PWM tiap loop

// ==================================================
// INPUT TRIM (untuk lurusin maju)
// ==================================================
// Kalau robot maju serong ke KANAN -> artinya ada X>0,
// coba TRIM_X positif kecil (0.02 ~ 2%).
// Kalau malah serong kiri, ganti jadi negatif.
float TRIM_X = 0.00f;   // mulai dulu dari 0.00, nanti tinggal diubah

float TRIM_R = 0.00f;   // kalau pas maju suka berputar, bisa pakai ini

int applyMinPwm(float cmd, int minPwm){
    if (cmd == 0) return 0;

    float sign = (cmd > 0) ? 1.0f : -1.0f;
    float mag  = fabs(cmd); // besarnya 0..255

    float out = minPwm + (255 - minPwm) * (mag / 255.0f);
    return (int)(sign * out);
}

float pidUpdate(PID &pid, float target, float meas) {
    float e = target - meas;         // error (tick/period)

    pid.integral += e * DT;
    // anti wind-up
    if (pid.integral > 200) pid.integral = 200;
    if (pid.integral < -200) pid.integral = -200;

    float d = (e - pid.prevError) / DT;
    pid.prevError = e;

    return pid.kp * e + pid.ki * pid.integral + pid.kd * d;
}


// ==================================================
// ACCELERATION LIMITER
// ==================================================
float limitAccel(float last, float now, float step){
    if(now > last + step) return last + step;
    if(now < last - step) return last - step;
    return now;
}

// ==================================================
// FILTERED SPEED
// ==================================================
float filteredVelocity[MOTOR_COUNT] = {0};
long prevEncoder[MOTOR_COUNT] = {0};
float motorCommand[MOTOR_COUNT] = {0};

// ==================================================
const unsigned long CONTROL_PERIOD = 5;  // 5 ms ~ 200 Hz
const float MAX_TICK = 80;
float Xs=0, Ys=0, Rs=0;

void setup(){
    Serial.begin(115200);

    pinMode(PPM_PIN,INPUT);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

    for(size_t i=0; i<MOTOR_COUNT; ++i){
        pinMode(ENC_PIN_A[i], INPUT_PULLUP);
        pinMode(ENC_PIN_B[i], INPUT_PULLUP);
    }

    attachInterrupt(digitalPinToInterrupt(ENC_PIN_A[0]), enc1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_A[1]), enc2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_A[2]), enc3ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_A[3]), enc4ISR, CHANGE);

    Serial.println("Feedforward + P-only + Trim + Encoders READY!");
}

// ==================================================
void loop(){
    static unsigned long last=0;
    if(millis()-last < CONTROL_PERIOD) return;
    last = millis();

    // ---------------------------
    // BACA REMOTE
    // ---------------------------
    int ch1=ppm[0], ch2=ppm[1], ch3=ppm[2], ch4=ppm[3];
    if(ch1<900||ch1>2100) ch1=1500;
    if(ch2<900||ch2>2100) ch2=1500;
    if(ch3<900||ch3>2100) ch3=1500;
    if(ch4<900||ch4>2100) ch4=1500;

    float X_raw=(ch1-1500)/500.0f;
    float Y_raw=(ch2-1500)/500.0f;
    float R_raw=(ch4-1500)/500.0f;
    float gas=constrain((ch3-1000)/1000.0f, 0, 1);

    // Aplikasikan TRIM di sini
    float X = X_raw - TRIM_X;
    float Y = Y_raw;
    float R = R_raw - TRIM_R;

    X = expo(deadzone(X));
    Y = expo(deadzone(Y));
    R = expo(deadzone(R));

    Xs = smooth(Xs, X, 0.2);
    Ys = smooth(Ys, Y, 0.2);
    Rs = smooth(Rs, R, 0.2);

    // ---------------------------
    // KINEMATIKA OMNI
    // ---------------------------
    float wheelMix[MOTOR_COUNT];
    wheelMix[0] = Ys - Xs - Rs;
    wheelMix[1] = Ys + Xs + Rs;
    wheelMix[2] = Ys - Xs + Rs;
    wheelMix[3] = Ys + Xs - Rs;

    float maxV = 0.0f;
    for(size_t i=0; i<MOTOR_COUNT; ++i){
        maxV = max(maxV, fabs(wheelMix[i]));
    }
    if(maxV>1){
        for(size_t i=0; i<MOTOR_COUNT; ++i){
            wheelMix[i] /= maxV;
        }
    }

    float targetTicks[MOTOR_COUNT];
    for(size_t i=0; i<MOTOR_COUNT; ++i){
        targetTicks[i] = wheelMix[i] * MAX_TICK * gas;
    }

    // ---------------------------
    // ENCODER SPEED PER PERIOD
    // ---------------------------
    long encoderSnapshot[MOTOR_COUNT];
    for(size_t i=0; i<MOTOR_COUNT; ++i){
        encoderSnapshot[i] = encoderCounts[i];
        float vel = encoderSnapshot[i] - prevEncoder[i];
        prevEncoder[i] = encoderSnapshot[i];
        filteredVelocity[i] = filteredVelocity[i]*0.8f + vel*0.2f;
    }

    // ---------------------------
    // FEEDFORWARD + P-ONLY
    // ---------------------------
    float limit = 255 * gas;
    for(size_t i=0; i<MOTOR_COUNT; ++i){
        float ff = FF_GAIN * targetTicks[i];
        float corr = pidUpdate(motorTune[i].pid, targetTicks[i], filteredVelocity[i]);
        motorCommand[i] = limitAccel(motorCommand[i], ff + corr, ACCEL_STEP);

        float compensated = motorCommand[i] * motorTune[i].compensation;
        float bounded = constrain(compensated, -limit, limit);
        int pwmCmd = applyMinPwm(bounded, motorTune[i].minPwm);

        driveMotor(MOTOR_PIN_L[i], MOTOR_PIN_R[i], pwmCmd);
    }

    // ---------------------------
    // DEBUG
    // ---------------------------
    Serial.print("XYR: ");
    Serial.print(Xs,3); Serial.print(",");
    Serial.print(Ys,3); Serial.print(",");
    Serial.print(Rs,3);

    Serial.print(" | ENC: ");
    for(size_t i=0; i<MOTOR_COUNT; ++i){
        Serial.print(filteredVelocity[i]);
        if(i<MOTOR_COUNT-1) Serial.print(",");
    }
    Serial.println();
}
