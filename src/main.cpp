#include <Arduino.h>
#include "microros_node.h"

/* 
 * --- SELURUH KODE ROBOT KAMU DI SINI ---
 * copy semua kode kontrol motor kamu
 * mulai dari PPM, PID, encoder, driveMotor, dll
 *
 * CATATAN:
 * Jangan hapus apa pun selain Serial debug
 */

// ===== Tambahan untuk mode autonoumous vs manual ====
bool autonomousMode = false;
float autoX=0, autoY=0, autoR=0;

// =========================================================
// SETUP
// =========================================================
void setup() {
    // setup robot kamu
    Serial.begin(115200);

    // Init semua seperti program kamu
    // PPM, encoder, motor pin, PID dsb…

    // Init micro-ROS Node
    microRosInit();
}

// =========================================================
// LOOP
// =========================================================
void loop() {

    // Spin micro-ROS (non-blocking)
    microRosSpinOnce();

    // Ambil cmd_vel dari Jetson
    getCmdVel(autoX, autoY, autoR);

    // Jika Jetson kirim cmd_vel besar → autonomous ON
    if (fabs(autoX)+fabs(autoY)+fabs(autoR) > 0.05f) {
        autonomousMode = true;
    }

    // Jika throttle PPM aktif → manual lebih tinggi prioritas
    if ( /* logika PPM kamu gas > 0, dll */ ) {
        autonomousMode = false;
    }

    // === OVERWRITE INPUT KINEMATIK ===
    if (autonomousMode) {
        // mapping ke joystick simulasi
        Xs = smooth(Xs, autoX, 0.1);
        Ys = smooth(Ys, autoY, 0.1);
        Rs = smooth(Rs, autoR, 0.1);
    } else {
        // gunakan input PPM kamu yang sekarang
        // Xs, Ys, Rs = hasil joystick kamu
    }

    // === Jalankan KODE ROBOT NORMAL ===
    // PID kamu
    // Kinematics kamu
    // Motor drive kamu

    // === Publish ENCODER ===
    publishEncoderData(encCount);
}
