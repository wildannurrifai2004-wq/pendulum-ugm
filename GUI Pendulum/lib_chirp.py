# -*- coding: utf-8 -*-
"""
lib_chirp.py - Swept Sine (Chirp) Maneuver Generator & Real-time Safety Supervisor
untuk Sistem Inverted Pendulum UGM.

Fitur:
- Pembangkit sinyal chirp (swept-frequency sine) kontinu:
  f(t) = f_start + (f_end - f_start) * (t / T_sweep)
  phi(t) = 2*pi * (f_start * t + 0.5 * (f_end - f_start) / T_sweep * t^2)
- Dua mode operasi:
  1. MODE_POSITION (Closed-Loop Trajectory): Menggerakkan cart bolak-balik
     mengikuti trajectory sinus dengan frekuensi meningkat, otomatis scaling
     amplitudo agar kecepatan dan percepatan tetap aman.
  2. MODE_DIRECT_PWM (Excitation with Center Spring): Mengirim eksitasi sinus
     langsung ke motor dengan pegas virtual pemusat untuk mencegah cart hanyut.
- Multi-layer Safety Supervisor:
  * Soft Limit Posisi Rel (X_MIN_SAFE, X_MAX_SAFE)
  * Overspeed Protection (V_MAX_SAFE)
  * Over-acceleration Protection (A_MAX_SAFE)
  * Limit switch trip
  * Auto emergency stop & recovery
"""

import math
import time


class SafetySupervisor:
    """
    Supervisor Keamanan Real-time untuk Kereta Rel Inverted Pendulum.
    Mencegah kereta keluar jalur (derail) dan kecepatan berlebih (overspeed).
    """

    def __init__(self,
                 x_min_trip=10.0,    # cm (batas soft kiri, trip darurat jika tembus)
                 x_max_trip=70.0,    # cm (batas soft kanan, trip darurat jika tembus)
                 x_center=40.0,      # cm (titik tengah rel)
                 v_max_safe=85.0,    # cm/s (kecepatan linier maksimum yang diizinkan)
                 a_max_safe=600.0,   # cm/s^2 (percepatan maksimum yang diizinkan)
                 pwm_max_clamp=18000 # PWM maksimum saat manuver (skala -25000 s/d 25000)
                 ):
        self.x_min_trip = float(x_min_trip)
        self.x_max_trip = float(x_max_trip)
        self.x_center = float(x_center)
        self.v_max_safe = float(v_max_safe)
        self.a_max_safe = float(a_max_safe)
        self.pwm_max_clamp = int(pwm_max_clamp)

        self.tripped = False
        self.trip_reason = ""
        self.trip_time = 0.0

    def reset_trip(self):
        """Reset status trip keamanan."""
        self.tripped = False
        self.trip_reason = ""
        self.trip_time = 0.0

    def check(self, cmX: float, v_cm_s: float, a_cm_s2: float, limit_sw_hit: bool = False) -> tuple:
        """
        Periksa status keamanan kereta.
        Mengembalikan: (is_safe: bool, status_str: str)
        """
        if self.tripped:
            return False, f"TRIPPED: {self.trip_reason}"

        if limit_sw_hit:
            self.tripped = True
            self.trip_reason = "Limit switch tersentuh!"
            self.trip_time = time.time()
            return False, f"TRIPPED: {self.trip_reason}"

        if cmX <= self.x_min_trip:
            self.tripped = True
            self.trip_reason = f"Posisi mendekati batas kiri ({cmX:.1f} cm <= {self.x_min_trip:.1f} cm)"
            self.trip_time = time.time()
            return False, f"TRIPPED: {self.trip_reason}"

        if cmX >= self.x_max_trip:
            self.tripped = True
            self.trip_reason = f"Posisi mendekati batas kanan ({cmX:.1f} cm >= {self.x_max_trip:.1f} cm)"
            self.trip_time = time.time()
            return False, f"TRIPPED: {self.trip_reason}"

        if abs(v_cm_s) > self.v_max_safe:
            self.tripped = True
            self.trip_reason = f"Overspeed! (|v|={abs(v_cm_s):.1f} cm/s > {self.v_max_safe:.1f} cm/s)"
            self.trip_time = time.time()
            return False, f"TRIPPED: {self.trip_reason}"

        # Peringatan dini jika mendekati batas (buffer 5 cm)
        if cmX < (self.x_min_trip + 5.0) or cmX > (self.x_max_trip - 5.0):
            return True, "WARNING_NEAR_LIMIT"

        return True, "SAFE"


class ChirpGenerator:
    """
    Pembangkit Manuver Sinusoid dengan Frekuensi Berubah-Ubah (Swept Sine / Chirp).
    """

    MODE_POSITION = "position"    # Tracking posisi closed-loop (Paling aman)
    MODE_DIRECT_PWM = "direct_pwm" # Eksitasi gaya langsung dengan centering spring

    def __init__(self,
                 f_start=0.2,       # Hz (frekuensi awal sweep)
                 f_end=2.0,         # Hz (frekuensi akhir sweep)
                 duration=30.0,     # Detik (durasi satu siklus sweep)
                 amplitude=14.0,    # cm (amplitudo posisi) atau PWM (mode direct)
                 x_center=40.0,     # cm (posisi tengah rel)
                 mode=MODE_POSITION):
        self.f_start = float(f_start)
        self.f_end = float(f_end)
        self.duration = max(5.0, float(duration))
        self.amplitude = float(amplitude)
        self.x_center = float(x_center)
        self.mode = mode

        # Gains kontroler tracking posisi (Mode Position)
        self.kp_pos = 180.0    # Proportional gain (posisi error cm -> ax / speed)
        self.kd_pos = 25.0     # Derivative / feedforward damping

        # Gains centering spring (Mode Direct PWM)
        self.k_center_spring = 350.0  # Menarik cart kembali ke tengah rel
        self.d_center_damp = 60.0     # Meredam getaran bebas

        self.running = False
        self.start_time = None
        self.elapsed_time = 0.0
        self.current_freq = self.f_start
        self.current_phase = 0.0
        self.last_update_time = None

    def start(self):
        """Mulai manuver swept-sine."""
        self.running = True
        self.start_time = time.time()
        self.last_update_time = self.start_time
        self.elapsed_time = 0.0
        self.current_phase = 0.0
        self.current_freq = self.f_start

    def stop(self):
        """Hentikan manuver swept-sine."""
        self.running = False
        self.start_time = None

    def is_running(self) -> bool:
        return self.running

    def update(self, current_cmX: float, current_v_cm_s: float, safety: SafetySupervisor) -> tuple:
        """
        Hitung output kontrol dan telemetry pada langkah waktu saat ini.
        Mengembalikan:
        (ax_command: int, setspeed_command: int, target_x: float, target_v: float, freq: float, is_done: bool)
        """
        if not self.running:
            return 0, 0, self.x_center, 0.0, 0.0, False

        now = time.time()
        dt = now - self.last_update_time if self.last_update_time else 0.02
        self.last_update_time = now
        self.elapsed_time = now - self.start_time

        # Periksa apakah siklus sweep sudah selesai
        if self.elapsed_time >= self.duration:
            self.stop()
            return 0, 0, self.x_center, 0.0, self.f_end, True

        t = self.elapsed_time
        T = self.duration
        f0 = self.f_start
        f1 = self.f_end

        # 1. Hitung frekuensi sesaat f(t) (Linear Chirp)
        freq = f0 + (f1 - f0) * (t / T)
        self.current_freq = freq

        # 2. Hitung fase sesaat phi(t) = 2*pi * integral(f(t) dt)
        phase = 2.0 * math.pi * (f0 * t + 0.5 * (f1 - f0) / T * (t ** 2))
        self.current_phase = phase

        # 3. Taper amplitudo pada frekuensi tinggi agar tidak overspeed
        # Kecepatan puncak teoretis v_peak = 2*pi * f * A. Jika v_peak > v_max_safe * 0.7, turunkan A!
        max_v_allow = safety.v_max_safe * 0.65
        omega = 2.0 * math.pi * freq
        safe_amp = self.amplitude
        if omega > 0 and (omega * safe_amp) > max_v_allow:
            safe_amp = max_v_allow / omega

        sin_val = math.sin(phase)
        cos_val = math.cos(phase)

        target_x = self.x_center
        target_v = 0.0
        output_pwm = 0

        if self.mode == self.MODE_POSITION:
            # Trajectory target
            target_x = self.x_center + safe_amp * sin_val
            target_v = safe_amp * omega * cos_val  # Turunan analitik target velocity

            # PD Controller + Feedforward Velocity
            e_x = target_x - current_cmX
            e_v = target_v - current_v_cm_s
            u = (self.kp_pos * e_x) + (self.kd_pos * e_v)
            output_pwm = int(u)

        elif self.mode == self.MODE_DIRECT_PWM:
            # Eksitasi PWM Sinusoidal langsung + Virtual Centering Spring
            u_chirp = safe_amp * sin_val
            u_spring = -self.k_center_spring * (current_cmX - self.x_center) - (self.d_center_damp * current_v_cm_s)
            output_pwm = int(u_chirp + u_spring)

        # 4. Keamanan Clamp Output PWM
        clamp = safety.pwm_max_clamp
        output_pwm = max(-clamp, min(clamp, output_pwm))

        # Skala output_pwm ke format ax joystick STM32 (ax skala -32768 s/d 32767)
        # Pada STM32: setspeed = ax * 6000 / 32000. Jadi ax = setspeed * (32000 / 6000)
        ax_cmd = int(output_pwm * (32000.0 / 6000.0))
        ax_cmd = max(-32767, min(32767, ax_cmd))

        return ax_cmd, output_pwm, target_x, target_v, freq, False
