"""
lib_metrics.py - Real-time Performance Metrics untuk Pendulum

Menghitung Rise Time, Settling Time, dan Overshoot dari sinyal theta (sudut)
saat mode Balance aktif.

Definisi:
- Rise Time    : waktu dari |theta| mulai turun (peak) hingga masuk ±5° pertama kali
- Settling Time: waktu dari awal balance hingga |theta| masuk dan TETAP di ±2° (band 2%)
- Overshoot    : nilai puncak |theta| dalam derajat setelah mode balance aktif
"""

import math
import time
import threading
from collections import deque


class PerformanceMetrics:
    """
    Kalkulator metrik performa real-time dari sinyal theta pendulum.

    Cara pakai:
        metrics = PerformanceMetrics()
        metrics.reset()                          # saat Balance ditekan
        metrics.update(theta_rad, timestamp)     # dipanggil tiap data masuk
        rt, st, os_ = metrics.get_current()     # ambil nilai terkini
        hist = metrics.get_history()             # untuk grafik
    """

    SETTLE_BAND_DEG  = 2.0   # ±2° dianggap "settled"
    SETTLE_DWELL_S   = 0.5   # durasi harus bertahan stabil di dalam band (detik) — diubah ke 1.0s
    RISE_TARGET_DEG  = 5.0   # threshold masuk "rise done"
    HISTORY_SIZE     = 300   # jumlah sample histori untuk grafik

    def __init__(self):
        self._lock = threading.Lock()
        self._reset_internal()

        # Histori untuk grafik (deque ukuran tetap)
        self._hist_time       = deque(maxlen=self.HISTORY_SIZE)
        self._hist_rise       = deque(maxlen=self.HISTORY_SIZE)
        self._hist_settling   = deque(maxlen=self.HISTORY_SIZE)
        self._hist_overshoot  = deque(maxlen=self.HISTORY_SIZE)

        # Nilai terkini (yang sudah selesai dihitung)
        self._rise_time_s     = float('nan')
        self._settling_time_s = float('nan')
        self._overshoot_deg   = float('nan')

        self._active = False   # apakah sedang dalam sesi balance

    # ------------------------------------------------------------------ #
    #  API publik                                                          #
    # ------------------------------------------------------------------ #

    def reset(self):
        """
        Reset semua state metrik dan siap menunggu kriteria LQR Catch (<= 15°).
        Dipanggil saat tombol START / Balance / Swing-up ditekan.
        """
        with self._lock:
            self._reset_internal()
            self._active = False  # Set False agar update() menunggu momen catch <= 15°

    def deactivate(self):
        """
        Nonaktifkan perhitungan (dipanggil saat STOP/RESET).
        Tidak menghapus histori — grafik tetap tampil.
        """
        with self._lock:
            self._active = False

    def update(self, theta_rad: float, timestamp: float = None, theta_dot: float = 0.0, state_main: int = None):
        """
        Terima satu sample theta baru dari STM32.

        Args:
            theta_rad  : sudut pendulum dalam radian (0 = tegak atas)
            timestamp  : waktu absolut (time.time()). Jika None, pakai time.time().
            theta_dot  : kecepatan sudut (rad/s)
            state_main : state FSM dari firmware STM32 (7 = PEND_BALANCE)
        """
        if timestamp is None:
            timestamp = time.time()

        with self._lock:
            theta_deg = math.degrees(abs(theta_rad))

            # Auto-activate t_start berdasarkan event asli state_main == 7 (BALANCE LQR)
            # atau kriteria catch resmi (|degree| <= 15° & |theta_dot| <= 4.0 rad/s)
            is_balance_state = (state_main == 7) if state_main is not None else False
            is_catch_condition = (theta_deg <= 15.0) and (abs(theta_dot) <= 4.0)

            # Jika belum aktif, HANYA aktifkan saat pendulum masuk kriteria catch LQR (<= 15°)
            if not self._active:
                if is_balance_state or is_catch_condition:
                    self._reset_internal()
                    self._active = True
                    self._t_start = timestamp  # Catat t_start TEPAT saat catch terjadi!
                    self._peak_deg = theta_deg
                else:
                    return

            # --- Catat waktu mulai sesi t_start murni saat Balance LQR aktif ---
            if self._t_start is None:
                self._t_start = timestamp
                self._peak_deg = theta_deg

            elapsed = max(0.0, timestamp - self._t_start)

            # --- Rise Time: waktu pertama kali |theta| <= RISE_TARGET_DEG (<= 5°) ---
            if self._rise_time_s is None or math.isnan(self._rise_time_s):
                if theta_deg <= self.RISE_TARGET_DEG:
                    self._rise_time_s = elapsed

            # Firmware STM32 keluar dari balance (state 5 STOP) jika |degree| > 20°.
            # Deaktivasi metrik agar fase jatuh bebas ke 180° tidak merusak overshoot!
            if theta_deg > 20.0:
                self._active = False
                return

            # --- Overshoot: simpangan puncak selama berada di area kontrol balance (<= 20°) ---
            if theta_deg <= 20.0:
                if math.isnan(self._overshoot_deg):
                    self._overshoot_deg = theta_deg
                elif theta_deg > self._overshoot_deg:
                    self._overshoot_deg = theta_deg

            # --- Settling Time: masuk band (<= 2°) dan bertahan selama >= 1.0 s ---
            if math.isnan(self._settling_time_s):
                if theta_deg <= self.SETTLE_BAND_DEG:
                    if self._t_enter_band is None:
                        self._t_enter_band = timestamp
                    elif (timestamp - self._t_enter_band) >= self.SETTLE_DWELL_S:
                        # Sudah stabil 0.5 s penuh dalam band (mencakup dwell time 1.0 s)
                        self._settling_time_s = max(0.0, timestamp - self._t_start)
                else:
                    # Keluar dari band sebelum 1.0s — reset timer
                    self._t_enter_band = None

            # --- Update histori setiap 5 sample (sekitar 0.1 s @ 50 Hz) ---
            self._sample_count += 1
            if self._sample_count % 5 == 0:
                self._hist_time.append(elapsed)
                rt  = self._rise_time_s    if (self._rise_time_s is not None and not math.isnan(self._rise_time_s))    else float('nan')
                st  = self._settling_time_s if (self._settling_time_s is not None and not math.isnan(self._settling_time_s)) else float('nan')
                ov  = self._overshoot_deg  if (self._overshoot_deg is not None and not math.isnan(self._overshoot_deg))  else float('nan')
                self._hist_rise.append(rt  if not math.isnan(rt)  else None)
                self._hist_settling.append(st  if not math.isnan(st)  else None)
                self._hist_overshoot.append(ov  if not math.isnan(ov) else None)

    def get_current(self):
        """
        Return (rise_time_s, settling_time_s, overshoot_deg) nilai terkini.
        Nilai float('nan') berarti belum selesai dihitung.
        """
        with self._lock:
            return (
                self._rise_time_s,
                self._settling_time_s,
                self._overshoot_deg
            )

    def get_history(self):
        """
        Return dict histori untuk plotting grafik.
        Key: 'time', 'rise', 'settling', 'overshoot'
        Value: list (bisa berisi None untuk nilai yang belum ada)
        """
        with self._lock:
            return {
                'time'      : list(self._hist_time),
                'rise'      : list(self._hist_rise),
                'settling'  : list(self._hist_settling),
                'overshoot' : list(self._hist_overshoot),
            }

    def is_active(self):
        with self._lock:
            return self._active

    # ------------------------------------------------------------------ #
    #  Internal                                                            #
    # ------------------------------------------------------------------ #

    def _reset_internal(self):
        """Reset state internal tanpa mengosongkan histori."""
        self._t_start          = None
        self._t_enter_band     = None
        self._peak_deg         = 0.0
        self._sample_count     = 0
        self._rise_time_s      = float('nan')
        self._settling_time_s  = float('nan')
        self._overshoot_deg    = float('nan')
