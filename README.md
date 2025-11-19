# 📌 **README.md – Omniwheel Robot Control (4 Wheel + Encoder + PID + RC PPM)**

# 🚀 Omniwheel Robot Controller

Sistem kendali robot omniwheel 4 roda menggunakan:

* **Motor PG36 + Encoder**
* **Kinematika Omni 45° (4 arah)**
* **PID Kecepatan per Roda (Closed-Loop Velocity Control)**
* **Feedforward + Deadzone Compensation**
* **Acceleration Limiter**
* **PPM Remote Control (RC Receiver)**
* **Arduino Mega 2560**

Robot dapat bergerak:

* Maju/Mundur
* Kiri/Kanan
* Rotasi CW/CCW
* Gerak diagonal
* Kombinasi translasi + rotasi

Dengan bantuan encoder + PID, robot tetap **lurus** meskipun motor tidak seimbang secara hardware.

---

# 📷 Konfigurasi Robot

### Layout roda (45° omni wheel)

```
   [M1]        [M2]
   Depan Kiri  Depan Kanan

   [M3]        [M4]
 Belakang Kiri Belakang Kanan
```

## Encoder Direction Table

| Motor | ENC_A | ENC_B | arah | DIR |
| ----- | ----- | ----- | ---- | --- |
| M1    | 19    | 22    | +    | +1  |
| M2    | 20    | 23    | -    | -1  |
| M3    | 21    | 24    | +    | +1  |
| M4    | 3     | 25    | -    | -1  |

---

# 🔧 **Fitur Utama Program**

### ✔ 1. **PPM Input**

Membaca sinyal RC (1–6 channel) dari receiver menggunakan interrupt.

### ✔ 2. **Kinematika Omni 4 Wheel (45°)**

Rumus:

```
M1 = +Y - X - R
M2 = +Y + X + R
M3 = +Y - X + R
M4 = +Y + X - R
```

Semua nilai dinormalisasi agar berada di [-1..1].

---

# ✔ 3. **Encoder Velocity Feedback**

Encoder dibaca via interrupt → dihitung selisih tick setiap 5 ms → difilter low-pass:

```
v = encoder_now - encoder_prev
filtered = 0.8 * last + 0.2 * v
```

---

# ✔ 4. **PID Kecepatan Per-Roda**

PID menghitung error antara target tick dan measured tick:

```cpp
error = target - measured
output = kp*e + ki*∫e + kd*de/dt
```

Setiap motor punya parameter PID berbeda jika mau tuning per motor.

---

# ✔ 5. **Feedforward Control**

Agar motor bisa mengikuti perintah dasar tanpa menunggu PID:

```cpp
PWM_ff = FF_GAIN * target_ticks
```

---

# ✔ 6. **Motor Deadzone Compensation**

Motor PG36 memiliki deadzone (mulai muter hanya jika PWM > 60–75).
Maka program memberi "offset" minimum PWM per motor:

```cpp
MIN_PWM1 = 70;
MIN_PWM2 = 60;
MIN_PWM3 = 70;
MIN_PWM4 = 60;
```

Fungsi:

```cpp
applyMinPwm()
```

memastikan motor mulai bergerak **pada PWM minimum yang tepat**.

---

# ✔ 7. **Acceleration Limiter (Anti Kick/Slip)**

```cpp
u = limitAccel(prev, now, step=8)
```

Agar perubahan PWM halus dan tidak “nyentak”.

---

# ✔ 8. **Motor Compensation Factor**

Jika ada motor yang terlalu cepat/lambat:

```cpp
K1 = 0.97f
K2 = 1.00f
...
```

---

# ✔ 9. **Trim Input RC**

Agar robot tetap lurus walau joystick tidak presisi:

```cpp
float TRIM_X = 0.00f;
float TRIM_R = 0.00f;
```

---

# 📁 **Struktur File**

```
/project
│
├── main.ino or main.cpp     ← Kode utama robot
└── README.md                ← Dokumentasi (file ini)
```

---

# ⚙️ **Cara Instalasi**

### 1. Gunakan board:

* Arduino Mega 2560 (disarankan)
* ESC/Motor Driver sesuai PG36 (BTS7960/IBT-2 recommended)

### 2. Wiring:

* Encoder ke interrupt pin: 2, 3, 19–21 (Mega)
* Motor driver ke pin PWM A0–A11
* PPM ke pin 18 (Interrupt)

### 3. Kompilasi

Gunakan Arduino IDE atau PlatformIO:

```
Board: Arduino Mega 2560  
Baud: 115200
```

---

# 🧪 Cara Kalibrasi Motor (PENTING)

## 1️⃣ Kalibrasi MIN_PWM

Untuk masing-masing motor:

1. Kirim PWM bertahap (0 → 255)
2. Catat nilai minimal di mana motor **mulai berputar**
3. Isi ke:

```
motorTune[i].minPwm = X;
```

## 2️⃣ Tuning PID

Mulai dari:

```
kp = 2.0
ki = 0.15
kd = 0
```

Jika robot masih serong:

* tambah **KI**
* kurangi **KP** jika goyang/oscillation
* tambah sedikit **KD** jika motor overshoot

---

# 🎯 Cara Test dan Debug

Gunakan Serial Monitor:

Robot mencetak:

```
XYR: 0.00,1.00,0.00 | ENC: 52,63,49,61
```

* Target berjalan lurus → semua encoder seharusnya mirip
* Jika salah satu lebih kecil → motor lemah → naikkan minPWM
* Jika lebih besar → turunkan compensation factor

---

# 📝 Catatan Tambahan

* Program sudah BEBAS blocking: semua dijalankan tiap 5 ms (200 Hz)
* Encoder + PID di-loop → robot sangat stabil
* Mendukung setiap arah gerakan + kombo rotasi

---

# 🤖 Author

Ikmal — Omniwheel Robotics Project
