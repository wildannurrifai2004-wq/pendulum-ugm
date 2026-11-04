# -*- coding: utf-8 -*-
import csv
import os
import time
import threading
import queue
from datetime import datetime


class RealtimeKinematicsEstimator:
    """
    Estimator turunan numerik (kecepatan dan percepatan) secara real-time
    menggunakan filter lolos-rendah (exponential smoothing) untuk meminimalkan derau kuantisasi.
    """

    def __init__(self, alpha_v=0.35, alpha_a=0.25):
        self.alpha_v = alpha_v
        self.alpha_a = alpha_a
        self.last_t = None
        self.last_x = None
        self.last_v = 0.0
        self.v_filt = 0.0
        self.a_filt = 0.0

    def reset(self):
        self.last_t = None
        self.last_x = None
        self.last_v = 0.0
        self.v_filt = 0.0
        self.a_filt = 0.0

    def update(self, t: float, x: float) -> tuple:
        if self.last_t is None:
            self.last_t = t
            self.last_x = x
            self.last_v = 0.0
            return 0.0, 0.0

        dt = t - self.last_t
        if dt <= 0.001:
            return self.v_filt, self.a_filt

        # Kecepatan linier sesaat
        v_raw = (x - self.last_x) / dt
        self.v_filt = (1.0 - self.alpha_v) * self.v_filt + self.alpha_v * v_raw

        # Percepatan linier sesaat
        a_raw = (self.v_filt - self.last_v) / dt
        self.a_filt = (1.0 - self.alpha_a) * self.a_filt + self.alpha_a * a_raw

        self.last_t = t
        self.last_x = x
        self.last_v = self.v_filt

        return self.v_filt, self.a_filt


class DataLogger:
    """
    Logger data performa, kinematika, dan aktuasi dari STM32 ke CSV.
    Mendukung logging real-time berkecepatan tinggi (~50 Hz) dengan antrian thread non-blocking.
    Mencakup: PWM, Posisi, Kecepatan, Percepatan, Frekuensi Chirp, dan Status Keamanan.
    """

    def __init__(self, base_dir="logs"):
        self.base_dir = base_dir
        os.makedirs(self.base_dir, exist_ok=True)

        self._lock = threading.Lock()
        self._recording = False
        self._file = None
        self._writer = None
        self._queue = queue.Queue()
        self._row_count = 0
        self._start_time = None
        self.current_stick_length = None
        self.session_prefix = "log"

        self.kinematics = RealtimeKinematicsEstimator()

        self._worker_thread = threading.Thread(
            target=self._worker_loop,
            daemon=True
        )
        self._worker_thread.start()

    # ---------------- API ----------------

    def is_recording(self) -> bool:
        with self._lock:
            return self._recording

    def set_recording(self, enabled: bool, prefix="log"):
        """
        Aktif/nonaktifkan rekam.
        Tiap kali ON akan buka file baru dengan timestamp & prefix.
        """
        with self._lock:
            if enabled == self._recording:
                return

            self.session_prefix = prefix
            if enabled:
                self.kinematics.reset()
                self._open_new_file()
            else:
                self._close_file()

            self._recording = enabled

    def toggle_recording(self, prefix="log"):
        with self._lock:
            new_state = not self._recording
        self.set_recording(new_state, prefix=prefix)

    def handle_sample(self, sample_tuple, extra_meta=None):
        """
        Dipanggil dari thread pembaca serial / GUI loop.
        sample_tuple: (logtick, degree, cmX, setspeed, r1, theta_dot, theta, x_center)
        extra_meta: dict opsional, misal {"chirp_freq": 1.25, "safety_status": "SAFE"}
        """
        now = time.time()
        self._queue.put((now, sample_tuple, extra_meta))

    # ---------------- internal ----------------

    def _open_new_file(self):
        if self._file is not None:
            self._close_file()

        now = time.time()
        self._start_time = now
        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
        filename = os.path.join(self.base_dir, f"{self.session_prefix}_{ts}.csv")
        self._file = open(filename, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._row_count = 0

        # Header lengkap untuk analisis Respon Dinamik, FCE, Deadband & Gaya Gesek
        self._writer.writerow([
            "datetime",
            "timestamp",
            "elapsed_s",
            "logtick",
            "degree",
            "cmX",
            "pwm",
            "motor_rpm",
            "cart_speed_cm_s",
            "cart_accel_cm_s2",
            "pendulum_omega_rad_s",
            "chirp_freq_hz",
            "safety_status",
            "reserved1",
            "reserved2",
            "reserved3",
            "reserved4"
        ])
        print(f"[DataLogger] Recording started -> {filename}")

    def _close_file(self):
        filename_to_export = None
        if self._file is not None:
            try:
                filename_to_export = self._file.name
                self._file.flush()
                self._file.close()
                print(f"[DataLogger] Recording finished -> {filename_to_export}")
            except Exception as e:
                print(f"[DataLogger] Close warning: {e}")
        self._file = None
        self._writer = None
        self._row_count = 0
        self._start_time = None

        if filename_to_export and os.path.exists(filename_to_export):
            # Export ke Excel secara otomatis di background thread
            def _export():
                try:
                    import sys
                    base_dir = os.path.dirname(os.path.abspath(__file__))
                    if base_dir not in sys.path:
                        sys.path.insert(0, base_dir)
                    from excel_exporter import process_csv_and_export_excel
                    process_csv_and_export_excel(filename_to_export, stick_length=self.current_stick_length)
                except Exception as e:
                    print(f"[EXCEL] Auto Export notice: {e}")
            threading.Thread(target=_export, daemon=True).start()

    def _worker_loop(self):
        """
        Loop penulis CSV di thread background.
        Selalu mengosongkan antrian agar memori tetap bersih.
        """
        while True:
            try:
                item = self._queue.get()   # blocking
                with self._lock:
                    if self._recording and self._writer is not None:
                        extra_meta = None
                        if isinstance(item, tuple):
                            if len(item) == 3:
                                recv_time, sample, extra_meta = item
                            elif len(item) == 2 and isinstance(item[0], float):
                                recv_time, sample = item
                            else:
                                recv_time = time.time()
                                sample = item
                        else:
                            recv_time = time.time()
                            sample = item

                        if self._start_time is None:
                            self._start_time = recv_time

                        elapsed_s = round(recv_time - self._start_time, 4)
                        dt_str = datetime.fromtimestamp(recv_time).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        ts_val = round(recv_time, 4)

                        sample_list = list(sample)
                        # sample_tuple: (logtick, degree, cmX, setspeed, r1, theta_dot, theta, x_center)
                        logtick = sample_list[0] if len(sample_list) > 0 else 0
                        degree  = sample_list[1] if len(sample_list) > 1 else 0.0
                        cmX     = float(sample_list[2]) if len(sample_list) > 2 else 0.0
                        raw_pwm = float(sample_list[3]) if len(sample_list) > 3 else 0.0
                        r1      = sample_list[4] if len(sample_list) > 4 else 0.0
                        theta_d = sample_list[5] if len(sample_list) > 5 else 0.0
                        r3      = sample_list[6] if len(sample_list) > 6 else 0.0
                        r4      = sample_list[7] if len(sample_list) > 7 else 0.0

                        # Hitung RPM motor
                        motor_rpm = round((abs(raw_pwm) / 25000.0) * 2400.0, 2)
                        if raw_pwm < 0:
                            motor_rpm = -motor_rpm

                        # Hitung kecepatan & percepatan linier kereta
                        v_cart, a_cart = self.kinematics.update(recv_time, cmX)

                        # Metadata chirp & keamanan
                        chirp_freq = 0.0
                        safety_str = "SAFE"
                        if extra_meta and isinstance(extra_meta, dict):
                            chirp_freq = float(extra_meta.get("chirp_freq", 0.0))
                            safety_str = str(extra_meta.get("safety_status", "SAFE"))

                        row = [
                            dt_str,
                            ts_val,
                            elapsed_s,
                            logtick,
                            round(float(degree), 3),
                            round(cmX, 3),
                            int(raw_pwm),
                            motor_rpm,
                            round(v_cart, 3),
                            round(a_cart, 3),
                            round(float(theta_d), 4),
                            round(chirp_freq, 3),
                            safety_str,
                            r1,
                            theta_d,
                            r3,
                            r4
                        ]

                        self._writer.writerow(row)
                        self._row_count += 1
                        # flush berkala per 50 baris (~1 detik @ 50 Hz)
                        if (self._row_count % 50) == 0:
                            self._file.flush()
            except Exception as e:
                print(f"[DataLogger Error] {e}")
