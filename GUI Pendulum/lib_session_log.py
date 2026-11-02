"""
lib_session_log.py - Session & Comparison Logger untuk Pendulum Monitor

Mencatat perbandingan setiap pengujian (panjang batang, massa, gain LQR,
Rise Time, Settling Time, Overshoot, Peak RPM) ke file CSV untuk analisis.
"""

import os
import csv
import time
import math
from datetime import datetime

class SessionLogger:
    def __init__(self, log_dir="logs", filename="session_comparison.csv"):
        self.log_dir = log_dir
        self.filepath = os.path.join(log_dir, filename)
        self.current_session = None
        self._ensure_csv_header()

    def _ensure_csv_header(self):
        os.makedirs(self.log_dir, exist_ok=True)
        if not os.path.exists(self.filepath) or os.path.getsize(self.filepath) == 0:
            with open(self.filepath, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Session_ID",
                    "Timestamp",
                    "Rod_Length_m",
                    "Rod_Mass_kg",
                    "K_TH",
                    "K_TH_D",
                    "K_X",
                    "K_X_D",
                    "Rise_Time_s",
                    "Settling_Time_s",
                    "Overshoot_deg",
                    "Peak_RPM",
                    "Status"
                ])

    def get_next_session_id(self):
        if not os.path.exists(self.filepath):
            return 1
        try:
            with open(self.filepath, mode='r', encoding='utf-8') as f:
                reader = csv.reader(f)
                rows = list(reader)
                if len(rows) <= 1:
                    return 1
                last_row = rows[-1]
                return int(last_row[0]) + 1
        except Exception:
            return 1

    def start_session(self, length_m, mass_kg, gains_dict):
        """Mencatat parameter awal saat START ditekan."""
        self.current_session = {
            "start_time": time.time(),
            "timestamp_str": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "length_m": length_m,
            "mass_kg": mass_kg,
            "gains": gains_dict.copy() if gains_dict else {},
            "peak_rpm": 0.0,
            "saved": False
        }

    def update_peak_rpm(self, rpm):
        if self.current_session:
            if rpm > self.current_session["peak_rpm"]:
                self.current_session["peak_rpm"] = rpm

    def save_session(self, metrics, reason="RESET"):
        """
        Menyimpan hasil metrik saat RESET atau tombol Save ditekan.
        Return: (session_id, filepath) atau None jika tidak ada sesi aktif.
        """
        if not self.current_session or self.current_session.get("saved", False):
            return None

        rt, st, os_ = metrics.get_current()

        def clean_val(val):
            if val is None or math.isnan(val):
                return "NaN"
            return f"{val:.4f}"

        session_id = self.get_next_session_id()
        g = self.current_session["gains"]

        row = [
            session_id,
            self.current_session["timestamp_str"],
            f"{self.current_session['length_m']:.4f}",
            f"{self.current_session['mass_kg']:.4f}",
            f"{g.get('K_TH', 0.0):.2f}",
            f"{g.get('K_TH_D', 0.0):.2f}",
            f"{g.get('K_X', 0.0):.2f}",
            f"{g.get('K_X_D', 0.0):.2f}",
            clean_val(rt),
            clean_val(st),
            clean_val(os_),
            f"{self.current_session['peak_rpm']:.1f}",
            reason
        ]

        try:
            with open(self.filepath, mode='a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(row)
            self.current_session["saved"] = True
            return session_id, self.filepath
        except Exception as e:
            print(f"[SessionLogger] Gagal menyimpan log: {e}")
            return None
