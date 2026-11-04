#ifndef SINGEN
#define SINGEN

#include <stdint.h>
#include <math.h>
#include "main.h"
#include "global.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * Pembangkit Sinyal Swept Sine (Chirp) untuk STM32
 * Sistem Identifikasi Inverted Pendulum UGM:
 * f(t) = f0 + (f1 - f0) * (t / T)
 * phi(t) = 2*pi * (f0 * t + 0.5 * (f1 - f0)/T * t^2)
 */

typedef struct {
    float f_start;       // Frekuensi awal sweep [Hz] (def: 0.2 Hz)
    float f_end;         // Frekuensi akhir sweep [Hz] (def: 2.0 Hz)
    float duration_s;    // Durasi satu sapuan sweep [s] (def: 30.0 s)
    float amplitude_cm;  // Amplitudo osilasi posisi [cm] (def: 14.0 cm)
    float x_center_cm;   // Titik tengah osilasi [cm] (def: 40.0 cm)
    float elapsed_s;     // Waktu berjalan [s]
    float current_freq;  // Frekuensi sesaat [Hz]
    float outsin;        // Target posisi sesaat [cm]
    float target_vel;    // Target kecepatan sesaat [cm/s]
    uint8_t active;      // Status aktif sweep
} ChirpConfig_t;

static ChirpConfig_t gChirp = {
    .f_start      = 0.2f,
    .f_end        = 2.0f,
    .duration_s   = 30.0f,
    .amplitude_cm = 14.0f,
    .x_center_cm  = 40.0f,
    .elapsed_s    = 0.0f,
    .current_freq = 0.2f,
    .outsin       = 40.0f,
    .target_vel   = 0.0f,
    .active       = 1
};

static inline void initChirp(float f0, float f1, float dur_s, float amp_cm, float x_mid) {
    gChirp.f_start      = f0;
    gChirp.f_end        = f1;
    gChirp.duration_s   = (dur_s > 2.0f) ? dur_s : 30.0f;
    gChirp.amplitude_cm = (amp_cm > 1.0f && amp_cm < 22.0f) ? amp_cm : 14.0f;
    gChirp.x_center_cm  = (x_mid > 20.0f && x_mid < 60.0f) ? x_mid : 40.0f;
    gChirp.elapsed_s    = 0.0f;
    gChirp.current_freq = f0;
    gChirp.outsin       = gChirp.x_center_cm;
    gChirp.target_vel   = 0.0f;
    gChirp.active       = 1;
}

static inline void loopSin(void) {
    const float dt = 0.005f; // loop delay task CONTROL = 5 ms (200 Hz)
    gChirp.elapsed_s += dt;

    if (gChirp.elapsed_s >= gChirp.duration_s) {
        // Reset siklus sweep jika selesai
        gChirp.elapsed_s = 0.0f;
    }

    float t = gChirp.elapsed_s;
    float T = gChirp.duration_s;
    float f0 = gChirp.f_start;
    float f1 = gChirp.f_end;

    // Frekuensi sesaat linier
    float freq = f0 + (f1 - f0) * (t / T);
    gChirp.current_freq = freq;

    // Fase integral
    float phase = 2.0f * (float)M_PI * (f0 * t + 0.5f * (f1 - f0) / T * (t * t));

    // Amplitudo aman: turunkan secara proporsional jika frekuensi tinggi agar tidak overspeed
    float omega = 2.0f * (float)M_PI * freq;
    float max_v_allowed = 60.0f; // cm/s
    float safe_amp = gChirp.amplitude_cm;
    if (omega * safe_amp > max_v_allowed) {
        safe_amp = max_v_allowed / omega;
    }

    // Posisi target sinusoida
    gChirp.outsin = gChirp.x_center_cm + safe_amp * sinf(phase);
    // Kecepatan target
    gChirp.target_vel = safe_amp * omega * cosf(phase);
}

#endif
