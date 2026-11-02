import os
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import sys
import threading
import pygame
import math
import time
import json
from threading import Lock

from lib_com import (open_serial, read_control_status, make_packet,
                     send_gains, send_reset, send_physics, send_swing_params,
                     find_available_ports, auto_detect_serial_port)
from lib_gui import draw_manual_controls, get_manual_hit, get_cmd_hit, MANUAL_AX_SPEED, CMD_BUTTONS
from lib_data import DataLogger
from lib_udp import UDPBroadcaster
from lib_metrics import PerformanceMetrics
from lib_session_log import SessionLogger

# ============================================================
# KONFIGURASI PERSISTEN
# ============================================================
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config_params.json")

PORT = "COM14"
BAUD = 115200
FPS  = 50

# Default gains LQR Standar 4-Gain (balance_controller)
DEFAULT_GAINS = {
    "K_TH"   : -1430.2188,
    "K_TH_D" : -66.2387,
    "K_X"    : 1.3608,
    "K_X_D"  : 2.1728,
    "K_X_INT": 0.0
}

# Default parameter fisik (sesuai lipmcontrol.h & CoM pendulum)
DEFAULT_PHYSICS = {
    "length_m"  : 0.1925,  # l_p [m] (jarak pivot ke pusat massa CoM)
    "mass_kg"   : 0.158    # m_p [kg]
}

# Default parameter swing-up & handover
DEFAULT_SWING = {
    "kE"       : 80.0,   # Gain energi swing-up
    "kx_swing" : 0.5,    # Penjaga rel saat ayunan
    "margin_E" : 1.05,   # Margin energi target (x E_des)
    "catch_deg": 15.0,   # Sudut tangkap LQR (|deg| < )
    "catch_vel": 4.0     # Kecepatan sudut tangkap (|rad/s| < )
}

def load_persistent_config():
    """Muat parameter terakhir dari file config_params.json jika ada."""
    gains = DEFAULT_GAINS.copy()
    physics = DEFAULT_PHYSICS.copy()
    swing = DEFAULT_SWING.copy()
    offset = 0.0
    port = "COM14"
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                saved = json.load(f)
                if "gains" in saved:
                    gains.update(saved["gains"])
                if "physics" in saved:
                    physics.update(saved["physics"])
                if "swing" in saved:
                    swing.update(saved["swing"])
                if "angle_offset_deg" in saved:
                    offset = float(saved["angle_offset_deg"])
                if "port" in saved:
                    port = str(saved["port"]).strip()
                print(f"[Config] Berhasil memuat konfigurasi dari {CONFIG_FILE}")
        except Exception as e:
            print(f"[Config] Peringatan membaca config: {e}")
    return gains, physics, swing, offset, port

# Rail limits
X_MIN_CM   = 0.0
X_MAX_CM   = 80.0
X_CENTER_CM = 40.0

# ============================================================
# SHARED STATE
# ============================================================
pendulum_state = {
    "cmX"             : X_CENTER_CM,
    "theta"           : math.pi,   # Default posisi gantung bawah (180°)
    "raw_degree"      : 180.0,
    "degree"          : 180.0,
    "x_center"        : 40.0,
    "running"         : False,
    "gains_ack"       : False,
    "gains_ack_values": None,
    "reset_ack"       : False,
    "physics_sent"    : False,
    "swing_ack"       : False,
    "swing_ack_values": None,
    "swing_sent"      : False,
    "setspeed"        : 0.0,     # kecepatan set point dari STM32
    "rpm"             : 0.0,     # RPM motor (dihitung dari setspeed)
}
state_lock = Lock()

current_gains = DEFAULT_GAINS.copy()
gains_lock    = Lock()

manual_state = {
    "ax"             : 0,
    "buttons"        : 0,
    "held_direction" : None,
    "held_cmd_key"   : None,
}
manual_lock = Lock()

# ============================================================
# GUI CONSTANTS
# ============================================================
DEFAULT_WINDOW_WIDTH  = 1120
DEFAULT_WINDOW_HEIGHT = 640
PANEL_WIDTH           = 300          # panel kanan (gains + physics)
STATUS_BAR_HEIGHT     = 36           # status bar bawah

# ── Colours ──────────────────────────────────────────────────
C_BG           = (18, 18, 28)
C_PANEL        = (28, 30, 45)
C_CHART_BORDER = (50, 55, 80)
C_TEXT         = (220, 220, 230)
C_TEXT_DIM     = (130, 130, 150)
C_STATUS_RUN   = (72, 199, 116)
C_STATUS_STOP  = (199, 72, 72)
C_REC_ON       = (220, 60, 60)
C_REC_OFF      = (70, 70, 90)
C_UDP_ON       = (60, 180, 220)
C_UDP_OFF      = (70, 70, 90)
C_BTN          = (55, 110, 175)
C_BTN_HOVER    = (75, 145, 215)
C_BTN_DISABLED = (55, 55, 70)
C_RAIL         = (170, 170, 180)
C_CART         = (90, 170, 255)
C_PENDULUM     = (240, 240, 255)
C_MASS         = (255, 90, 80)
# Metrics text colours
C_RISE         = (255, 220, 50)    # kuning
C_SETTLING     = (50, 220, 220)    # cyan
C_OVERSHOOT    = (255, 90, 90)     # merah-muda
# RPM
C_RPM_LOW      = (80, 220, 120)    # hijau (RPM rendah)
C_RPM_MED      = (255, 200, 50)    # kuning (RPM sedang)
C_RPM_HIGH     = (255, 80, 80)     # merah (RPM tinggi)
RPM_MAX        = 2400.0            # RPM maksimum motor (2400 RPM, 200 PPR)
SETSPEED_MAX   = 25000.0           # Skala maksimum setspeed / PWM dari STM32


# ============================================================
# HELPER CLASSES
# ============================================================
class Button:
    def __init__(self, x, y, w, h, text, enabled=True,
                 color_on=None, color_off=None, custom_color=None, custom_hover=None):
        self.rect         = pygame.Rect(x, y, w, h)
        self.text         = text
        self.enabled      = enabled
        self.hovered      = False
        self.color_on     = color_on
        self.color_off    = color_off
        self.custom_color = custom_color
        self.custom_hover = custom_hover

    def draw(self, screen, font):
        if not self.enabled:
            color = C_BTN_DISABLED
        elif self.hovered:
            color = self.custom_hover if self.custom_hover else C_BTN_HOVER
        else:
            if self.color_on and "ON" in self.text: color = self.color_on
            elif self.color_off and "OFF" in self.text: color = self.color_off
            elif self.custom_color: color = self.custom_color
            else: color = C_BTN
        pygame.draw.rect(screen, color,   self.rect, border_radius=6)
        pygame.draw.rect(screen, C_TEXT,  self.rect, 1, border_radius=6)
        surf = font.render(self.text, True, C_TEXT)
        screen.blit(surf, surf.get_rect(center=self.rect.center))

    def update_hover(self, pos):
        self.hovered = self.rect.collidepoint(pos) and self.enabled

    def is_clicked(self, pos):
        return self.enabled and self.rect.collidepoint(pos)


class InputField:
    def __init__(self, x, y, w, h, label, default_value, enabled=True):
        self.rect    = pygame.Rect(x, y, w, h)
        self.label   = label
        self.value   = str(default_value)
        self.enabled = enabled
        self.active  = False

    def draw(self, screen, font_label, font_input):
        lbl = font_label.render(self.label, True, C_TEXT_DIM)
        screen.blit(lbl, (self.rect.x, self.rect.y - 18))
        bg = (65, 68, 90) if self.active else (42, 45, 65)
        if not self.enabled: bg = (32, 34, 48)
        pygame.draw.rect(screen, bg, self.rect, border_radius=4)
        border = C_BTN if self.active else (70, 75, 100)
        pygame.draw.rect(screen, border, self.rect, 2, border_radius=4)
        surf = font_input.render(self.value, True, C_TEXT)
        screen.blit(surf, surf.get_rect(midleft=(self.rect.x + 8, self.rect.centery)))

    def handle_event(self, event):
        if not self.enabled: return False
        if event.type == pygame.MOUSEBUTTONDOWN:
            self.active = self.rect.collidepoint(event.pos)
        elif event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_BACKSPACE:
                self.value = self.value[:-1]
            elif event.key == pygame.K_RETURN:
                self.active = False
            elif event.unicode.isalnum() or event.unicode in '0123456789.-_/:':
                if len(self.value) < 25:
                    self.value += event.unicode
            return True
        return False

    def get_float(self, default=0.0):
        try:    return float(self.value)
        except: return default


# ============================================================
# MAIN GUI CLASS
# ============================================================
class PendulumMonitor:

    def __init__(self):
        pygame.init()
        self.width = DEFAULT_WINDOW_WIDTH
        self.height = DEFAULT_WINDOW_HEIGHT
        self.panel_width = PANEL_WIDTH
        self.main_width = self.width - self.panel_width
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        pygame.display.set_caption("Pendulum Monitor — Live Tuning + Physics + Metrics")

        self.clock      = pygame.time.Clock()
        self.font_large = pygame.font.SysFont("Arial", 22, bold=True)
        self.font_med   = pygame.font.SysFont("Arial", 16)
        self.font_small = pygame.font.SysFont("Arial", 14)
        self.font_tiny  = pygame.font.SysFont("Arial", 12)
        self.font_input = pygame.font.SysFont("Consolas", 14)

        self.data_logger     = DataLogger(base_dir="logs")
        self.udp_broadcaster = UDPBroadcaster(
            broadcast_ip="192.168.1.255",
            port=5000
        )
        self.metrics = PerformanceMetrics()
        self.session_logger = SessionLogger(log_dir="logs")
        self.session_notice = ""
        self.session_notice_time = 0.0

        # Opsi Perekaman (MANUAL vs AUTO 10S)
        self.auto_rec_10s = False          # False = MANUAL (jalan terus), True = AUTO 10S saat stabil
        self.auto_rec_triggered = False   # Flag apakah rekam 10s sudah dipicu
        self.auto_rec_start_time = None   # Timestamp awal rekam 10s

        self.saved_gains, self.saved_physics, self.saved_swing, self.angle_offset_deg, self.port = load_persistent_config()
        self.active_tab      = "lqr"
        self.swing_sent      = False
        self.swing_ack_time  = 0.0
        with gains_lock:
            current_gains.update(self.saved_gains)

        self._create_ui_elements()

        self.serial           = None
        self.serial_connected = False
        self.thread_rx        = None
        self.thread_tx        = None

        self.running         = True
        self.gains_sent      = False
        self.gains_ack_time  = 0

        self.tx_debug      = {"count": 0, "last_ax": 0, "last_buttons": 0}
        self.tx_debug_lock = Lock()

    # ─────────────────────────────────────────────────────────
    #  UI CREATION & RESPONSIVE LAYOUT
    # ─────────────────────────────────────────────────────────
    def _create_ui_elements(self):
        self._update_layout()

    def _update_layout(self):
        self.main_width = self.width - self.panel_width
        px = self.main_width + 15   # panel x start

        if not hasattr(self, 'inputs'):
            # Tab buttons
            self.btn_tab_lqr   = Button(px,       12, 132, 28, "LQR & Fisika", custom_color=(45, 110, 160))
            self.btn_tab_swing = Button(px + 138, 12, 132, 28, "Swing-Up",    custom_color=(50, 55, 75))

            # ── Gains inputs (LQR Standar 4 Gain) ──
            self.inputs = {
                "K_TH"   : InputField(px, 72,  270, 26, "K_TH — Sudut (rad)",            self.saved_gains["K_TH"]),
                "K_TH_D" : InputField(px, 124, 270, 26, "K_TH_D — Kec. Sudut (rad/s)",   self.saved_gains["K_TH_D"]),
                "K_X"    : InputField(px, 176, 270, 26, "K_X — Posisi Kereta (cm)",      self.saved_gains["K_X"]),
                "K_X_D"  : InputField(px, 228, 270, 26, "K_X_D — Kec. Kereta (cm/s)",   self.saved_gains["K_X_D"]),
            }
            self.btn_apply = Button(px, 265, 270, 30, "Apply Gains")

            # ── Parameter Fisik inputs ──
            self.inputs_physics = {
                "length_m" : InputField(px, 332, 270, 26,
                                        "Panjang CoM Pendulum l (m)",
                                        self.saved_physics["length_m"]),
                "mass_kg"  : InputField(px, 384, 270, 26,
                                        "Massa Pendulum m (kg)",
                                        self.saved_physics["mass_kg"]),
            }
            self.btn_send_physics = Button(px, 420, 270, 30,
                                           "Kirim Parameter Fisik")

            # ── Parameter Swing-Up & Handover inputs ──
            self.inputs_swing = {
                "kE"       : InputField(px, 72,  270, 26, "kE — Gain Energi Swing-Up",        self.saved_swing["kE"]),
                "kx_swing" : InputField(px, 124, 270, 26, "kx_swing — Penjaga Rel Kereta",    self.saved_swing["kx_swing"]),
                "margin_E" : InputField(px, 176, 270, 26, "Margin Energi Target (x E_des)",   self.saved_swing["margin_E"]),
                "catch_deg": InputField(px, 228, 270, 26, "Sudut Tangkap (|deg| < )",         self.saved_swing["catch_deg"]),
                "catch_vel": InputField(px, 280, 270, 26, "Kec. Tangkap (|rad/s| < )",        self.saved_swing["catch_vel"]),
            }
            self.btn_send_swing = Button(px, 322, 270, 32, "Kirim Parameter Swing-Up")

            self.btn_start = Button(px,       475, 85, 34, "START")
            self.btn_save  = Button(px + 90,  475, 90, 34, "SAVE LOG")
            self.btn_reset = Button(px + 185, 475, 85, 34, "RESET")

            self.input_port   = InputField(165, 12, 85, 30, "", self.port)
            self.btn_reconnect = Button(258, 12, 95, 30, "CONNECT", custom_color=(45, 110, 160))

            self.btn_rec_mode = Button(self.main_width - 390, 12, 130, 30, "REC: MANUAL",
                                       custom_color=(50, 55, 75))
            self.btn_udp      = Button(self.main_width - 250, 12, 115, 30, "UDP OFF",
                                       color_on=C_UDP_ON, color_off=C_UDP_OFF)
            self.btn_record   = Button(self.main_width - 125, 12, 115, 30, "REC OFF",
                                       color_on=C_REC_ON, color_off=C_REC_OFF)
        else:
            self.btn_tab_lqr.rect.x   = px
            self.btn_tab_swing.rect.x = px + 138

            for inp in self.inputs.values():
                inp.rect.x = px
            self.btn_apply.rect.x = px

            for inp in self.inputs_physics.values():
                inp.rect.x = px
            self.btn_send_physics.rect.x = px

            for inp in self.inputs_swing.values():
                inp.rect.x = px
            self.btn_send_swing.rect.x = px

            self.btn_start.rect.x = px
            self.btn_save.rect.x  = px + 90
            self.btn_reset.rect.x = px + 185

            self.input_port.rect.x    = 165
            self.btn_reconnect.rect.x = 258

            self.btn_rec_mode.rect.x = self.main_width - 390
            self.btn_udp.rect.x      = self.main_width - 250
            self.btn_record.rect.x   = self.main_width - 125

    # ─────────────────────────────────────────────────────────
    #  SERIAL SETUP
    # ─────────────────────────────────────────────────────────
    def setup_serial(self):
        """Buka port serial dengan fallback auto-detect & offline mode."""
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None

        port_to_try = self.input_port.value.strip() if hasattr(self, 'input_port') else self.port
        if not port_to_try:
            port_to_try = self.port
        self.port = port_to_try

        print(f"[Serial] Membuka port {port_to_try} @ {BAUD}...")
        try:
            self.serial = open_serial(port_to_try, BAUD)
            print(f"[Serial] Berhasil terhubung ke: {port_to_try} @ {BAUD}")
        except Exception as e1:
            print(f"[Serial] Port {port_to_try} tidak dapat dibuka ({e1})")
            auto_port = auto_detect_serial_port(preferred_port=port_to_try)
            if auto_port and auto_port != port_to_try:
                print(f"[Serial] Auto-detect mencoba port terdeteksi: {auto_port}...")
                try:
                    self.serial = open_serial(auto_port, BAUD)
                    self.port = auto_port
                    if hasattr(self, 'input_port'):
                        self.input_port.value = auto_port
                    self.save_persistent_config()
                    print(f"[Serial] Berhasil terhubung ke port alternatif: {auto_port}")
                except Exception as e2:
                    print(f"[Serial] Gagal membuka port alternatif {auto_port}: {e2}")
                    self.serial = None
            else:
                self.serial = None

        if self.serial is not None and getattr(self.serial, "is_open", False):
            self.serial_connected = True

            if self.thread_tx is None or not self.thread_tx.is_alive():
                self.thread_tx = threading.Thread(
                    target=self.manual_sender,
                    args=(FPS,),
                    daemon=True
                )
                self.thread_tx.start()

            if self.thread_rx is None or not self.thread_rx.is_alive():
                self.thread_rx = threading.Thread(
                    target=read_control_status,
                    args=(self.serial,),
                    kwargs={
                        "callback"          : self.on_control_status,
                        "ack_callback"      : self.on_gains_ack,
                        "reset_ack_callback": self.on_reset_ack,
                        "swing_ack_callback": self.on_swing_ack,
                        "debug"             : False
                    },
                    daemon=True
                )
                self.thread_rx.start()

            self.session_notice = f"✓ Serial Terhubung: {self.port}"
            self.session_notice_time = time.time()
            return True
        else:
            self.serial_connected = False
            available = find_available_ports()
            avail_str = ", ".join([p[0] for p in available]) if available else "Tidak ada"
            self.session_notice = f"[!] Serial Offline (Target: {port_to_try} | Ada: {avail_str})"
            self.session_notice_time = time.time()
            print(f"[Serial Warning] Port tidak terhubung. (Target: {port_to_try}, Port Ada: {avail_str}). Berjalan dalam Offline Mode.")
            return False

    def manual_sender(self, fps=50):
        """Thread TX: kirim state tombol GUI ke STM32 (format paket 0x01)."""
        period = 1.0 / fps
        seq    = 0
        last_print = 0
        while self.running:
            ser = self.serial
            if ser is None or not getattr(ser, "is_open", False):
                time.sleep(0.1)
                continue

            with manual_lock:
                ax      = manual_state["ax"]
                buttons = manual_state["buttons"]

            pkt = make_packet(seq, ax, 0, 0, 0, buttons)
            try:
                ser.write(pkt)
            except Exception as e:
                print(f"[TX Warning] Error mengirim serial: {e}")
                time.sleep(0.5)
                continue

            seq = (seq + 1) & 0xFF

            with self.tx_debug_lock:
                self.tx_debug["count"]        += 1
                self.tx_debug["last_ax"]       = ax
                self.tx_debug["last_buttons"]  = buttons

            now = time.time()
            if now - last_print > 0.5:
                print(f"[TX] seq={seq:3d} ax={ax:6d} buttons={buttons} pkt={pkt.hex()}")
                last_print = now

            time.sleep(period)

    # ─────────────────────────────────────────────────────────
    #  CALLBACKS
    # ─────────────────────────────────────────────────────────
    def on_control_status(self, sample_tuple):
        """Callback data dari STM32."""
        logtick, degree, cmX, setspeed, r1, theta_dot, theta, x_center = sample_tuple

        # Koreksi sudut dengan offset kalibrasi tare
        calibrated_degree = degree - self.angle_offset_deg
        # Normalisasi ke rentang [-180, 180] secara aman dari infinite loop (Inf/NaN)
        if math.isnan(calibrated_degree) or math.isinf(calibrated_degree):
            calibrated_degree = 180.0
        else:
            calibrated_degree = (calibrated_degree + 180.0) % 360.0 - 180.0

        calibrated_theta = math.radians(calibrated_degree)
        rpm_value = (abs(setspeed) / SETSPEED_MAX) * RPM_MAX

        with state_lock:
            pendulum_state["cmX"]        = cmX
            pendulum_state["raw_degree"] = degree
            pendulum_state["degree"]     = calibrated_degree
            pendulum_state["theta"]      = calibrated_theta
            pendulum_state["x_center"]   = x_center
            pendulum_state["setspeed"]   = setspeed
            pendulum_state["rpm"]        = rpm_value

        # Umpan ke metrics dan session logger
        now = time.time()
        self.metrics.update(calibrated_theta, now, theta_dot=theta_dot)
        self.session_logger.update_peak_rpm(rpm_value)

        # Logika Auto 10s Perekaman saat stabil
        if self.auto_rec_10s:
            rt, st, ov = self.metrics.get_current()
            if not math.isnan(st):
                if not self.auto_rec_triggered and not self.data_logger.is_recording():
                    try:
                        self.data_logger.current_stick_length = self.inputs_physics["length_m"].get_float(DEFAULT_PHYSICS["length_m"])
                    except Exception:
                        pass
                    self.data_logger.set_recording(True)
                    self.auto_rec_triggered = True
                    self.auto_rec_start_time = now
                    self.session_notice = "✓ Stabil — Rekam Auto 10s Dimulai!"
                    self.session_notice_time = now

            if self.auto_rec_triggered and self.data_logger.is_recording():
                elapsed_rec = now - self.auto_rec_start_time
                remaining = max(0.0, 10.0 - elapsed_rec)
                self.btn_record.text = f"REC ({remaining:.1f}s)"
                if elapsed_rec >= 10.0:
                    self.data_logger.set_recording(False)
                    self.btn_record.text = "REC OFF"
                    self.auto_rec_triggered = False
                    self.session_notice = "✓ Auto 10s Selesai — File Log & Excel Tersimpan!"
                    self.session_notice_time = now

        calibrated_sample = (logtick, calibrated_degree, cmX, setspeed, r1, theta_dot, calibrated_theta, x_center)
        self.data_logger.handle_sample(calibrated_sample)
        self.udp_broadcaster.send_control_status(calibrated_sample)

    def on_gains_ack(self, gains_tuple):
        K_TH, K_TH_D, K_X, K_X_D, K_X_INT = gains_tuple
        with state_lock:
            pendulum_state["gains_ack"]        = True
            pendulum_state["gains_ack_values"] = gains_tuple
        self.gains_ack_time = time.time()
        print(f"[ACK] Gains confirmed: K_TH={K_TH:.1f}, K_X={K_X:.2f}")

    def on_reset_ack(self, status):
        with state_lock:
            pendulum_state["reset_ack"] = True
            pendulum_state["running"]   = False
        print(f"[ACK] Reset confirmed by STM32 (status={status})")

    def on_swing_ack(self, swing_tuple):
        kE, kx_swing, margin_E, catch_deg, catch_vel = swing_tuple
        with state_lock:
            pendulum_state["swing_ack"]        = True
            pendulum_state["swing_ack_values"] = swing_tuple
        self.swing_ack_time = time.time()
        print(f"[ACK] Swing params confirmed: kE={kE:.1f}, kx={kx_swing:.2f}, margin={margin_E:.2f}, deg={catch_deg:.1f}")

    # ─────────────────────────────────────────────────────────
    #  EVENT HANDLING
    # ─────────────────────────────────────────────────────────
    def handle_events(self):
        mouse_pos = pygame.mouse.get_pos()

        for btn in (self.btn_apply, self.btn_start, self.btn_save, self.btn_reset,
                    self.btn_record, self.btn_udp, self.btn_rec_mode, self.btn_send_physics,
                    self.btn_send_swing, self.btn_tab_lqr, self.btn_tab_swing, self.btn_reconnect):
            btn.update_hover(mouse_pos)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.VIDEORESIZE:
                self.width = max(800, event.w)
                self.height = max(500, event.h)
                self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
                self._update_layout()

            active_inps = (list(self.inputs.values()) + list(self.inputs_physics.values())) if self.active_tab == "lqr" else list(self.inputs_swing.values())
            for inp in active_inps + [self.input_port]:
                inp.handle_event(event)

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if self.btn_tab_lqr.is_clicked(mouse_pos):
                    self.active_tab = "lqr"
                elif self.btn_tab_swing.is_clicked(mouse_pos):
                    self.active_tab = "swing"
                elif self.btn_reconnect.is_clicked(mouse_pos):
                    self.setup_serial()
                elif self.btn_send_swing.is_clicked(mouse_pos):
                    self.send_swing_params_action()
                elif self.btn_apply.is_clicked(mouse_pos):
                    self.apply_gains()
                elif self.btn_start.is_clicked(mouse_pos):
                    self.start_system()
                elif self.btn_save.is_clicked(mouse_pos):
                    self.save_session_manual()
                elif self.btn_reset.is_clicked(mouse_pos):
                    self.reset_system()
                elif self.btn_send_physics.is_clicked(mouse_pos):
                    self.send_physics_params()
                elif self.btn_rec_mode.is_clicked(mouse_pos):
                    self.auto_rec_10s = not self.auto_rec_10s
                    self.auto_rec_triggered = False
                    if self.auto_rec_10s:
                        self.btn_rec_mode.text = "REC: AUTO 10S"
                        self.btn_rec_mode.custom_color = (130, 80, 180)
                        self.session_notice = "Mode Rekam: AUTO 10S saat stabil"
                    else:
                        self.btn_rec_mode.text = "REC: MANUAL"
                        self.btn_rec_mode.custom_color = (50, 55, 75)
                        self.session_notice = "Mode Rekam: MANUAL (Jalan Terus)"
                    self.session_notice_time = time.time()
                elif self.btn_record.is_clicked(mouse_pos):
                    try:
                        self.data_logger.current_stick_length = self.inputs_physics["length_m"].get_float(DEFAULT_PHYSICS["length_m"])
                    except Exception:
                        pass
                    self.data_logger.toggle_recording()
                    is_rec = self.data_logger.is_recording()
                    self.btn_record.text = "REC ON" if is_rec else "REC OFF"
                elif self.btn_udp.is_clicked(mouse_pos):
                    self.udp_broadcaster.toggle()
                    is_udp = self.udp_broadcaster.enabled
                    self.btn_udp.text = "UDP ON" if is_udp else "UDP OFF"

                # Tombol kendali manual
                direction = get_manual_hit(mouse_pos)
                if direction is not None:
                    with manual_lock:
                        manual_state["ax"] = (
                            -MANUAL_AX_SPEED if direction == "left" else MANUAL_AX_SPEED
                        )
                        manual_state["held_direction"] = direction

                cmd_key = get_cmd_hit(mouse_pos)
                if cmd_key is not None:
                    with manual_lock:
                        manual_state["buttons"]       = CMD_BUTTONS[cmd_key]["value"]
                        manual_state["held_cmd_key"]  = cmd_key
                    # Reset metrik saat Balance atau Swing-up ditekan
                    if cmd_key in ("balance", "swingup"):
                        self.metrics.reset()
                        print(f"[Metrics] Sesi {cmd_key.capitalize()} baru — metrik direset")

            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                with manual_lock:
                    manual_state["ax"]             = 0
                    manual_state["buttons"]        = 0
                    manual_state["held_direction"] = None
                    manual_state["held_cmd_key"]   = None

    # ─────────────────────────────────────────────────────────
    #  ACTIONS
    # ─────────────────────────────────────────────────────────
    def save_persistent_config(self):
        """Simpan konfigurasi parameter ke file config_params.json."""
        try:
            cfg = {
                "port": self.input_port.value.strip() if hasattr(self, 'input_port') else self.port,
                "gains": {
                    "K_TH"   : self.inputs["K_TH"].get_float(DEFAULT_GAINS["K_TH"]),
                    "K_TH_D" : self.inputs["K_TH_D"].get_float(DEFAULT_GAINS["K_TH_D"]),
                    "K_X"    : self.inputs["K_X"].get_float(DEFAULT_GAINS["K_X"]),
                    "K_X_D"  : self.inputs["K_X_D"].get_float(DEFAULT_GAINS["K_X_D"]),
                },
                "physics": {
                    "length_m": self.inputs_physics["length_m"].get_float(DEFAULT_PHYSICS["length_m"]),
                    "mass_kg" : self.inputs_physics["mass_kg"].get_float(DEFAULT_PHYSICS["mass_kg"]),
                },
                "swing": {
                    "kE"       : self.inputs_swing["kE"].get_float(DEFAULT_SWING["kE"]),
                    "kx_swing" : self.inputs_swing["kx_swing"].get_float(DEFAULT_SWING["kx_swing"]),
                    "margin_E" : self.inputs_swing["margin_E"].get_float(DEFAULT_SWING["margin_E"]),
                    "catch_deg": self.inputs_swing["catch_deg"].get_float(DEFAULT_SWING["catch_deg"]),
                    "catch_vel": self.inputs_swing["catch_vel"].get_float(DEFAULT_SWING["catch_vel"]),
                },
                "angle_offset_deg": round(self.angle_offset_deg, 3)
            }
            with open(CONFIG_FILE, "w") as f:
                json.dump(cfg, f, indent=4)
        except Exception as e:
            print(f"[Config] Gagal menyimpan ke {CONFIG_FILE}: {e}")

    def send_swing_params_action(self, silent=False):
        """Kirim parameter swing-up & handover ke STM32 via USB CDC."""
        kE        = self.inputs_swing["kE"].get_float(DEFAULT_SWING["kE"])
        kx_swing  = self.inputs_swing["kx_swing"].get_float(DEFAULT_SWING["kx_swing"])
        margin_E  = self.inputs_swing["margin_E"].get_float(DEFAULT_SWING["margin_E"])
        catch_deg = self.inputs_swing["catch_deg"].get_float(DEFAULT_SWING["catch_deg"])
        catch_vel = self.inputs_swing["catch_vel"].get_float(DEFAULT_SWING["catch_vel"])

        # Validasi batas fisis
        if not (5.0 <= kE <= 3000.0):
            if not silent: print(f"[ERROR] kE tidak valid: {kE} (harus 5–3000)")
            return
        if not (0.0 <= kx_swing <= 5.0):
            if not silent: print(f"[ERROR] kx_swing tidak valid: {kx_swing} (harus 0–5.0)")
            return
        if not (0.8 <= margin_E <= 1.5):
            if not silent: print(f"[ERROR] margin_E tidak valid: {margin_E} (harus 0.8–1.5)")
            return
        if not (3.0 <= catch_deg <= 30.0):
            if not silent: print(f"[ERROR] catch_deg tidak valid: {catch_deg} (harus 3–30 deg)")
            return
        if not (0.5 <= catch_vel <= 10.0):
            if not silent: print(f"[ERROR] catch_vel tidak valid: {catch_vel} (harus 0.5–10.0 rad/s)")
            return

        self.save_persistent_config()

        if self.serial and getattr(self.serial, "is_open", False):
            for attempt in range(3):
                try:
                    send_swing_params(self.serial, kE, kx_swing, margin_E, catch_deg, catch_vel, seq=attempt)
                except Exception as e:
                    print(f"[TX Error] Swing send failed: {e}")
                time.sleep(0.05)
            self.swing_sent = True
            with state_lock:
                pendulum_state["swing_ack"] = False
                pendulum_state["swing_sent"] = True
            if not silent:
                print(f"[Swing] Terkirim: kE={kE:.1f}, kx={kx_swing:.2f}, margin={margin_E:.2f}, deg={catch_deg:.1f}, vel={catch_vel:.2f}")
        else:
            if not silent:
                print("[Warning] Serial tidak terhubung — parameter swing disimpan secara lokal.")

    def apply_gains(self, silent=False):
        with state_lock:
            if pendulum_state["running"]:
                if not silent:
                    print("Cannot apply gains while running! Stop first.")
                return

        with gains_lock:
            current_gains["K_TH"]    = self.inputs["K_TH"].get_float(DEFAULT_GAINS["K_TH"])
            current_gains["K_TH_D"]  = self.inputs["K_TH_D"].get_float(DEFAULT_GAINS["K_TH_D"])
            current_gains["K_X"]     = self.inputs["K_X"].get_float(DEFAULT_GAINS["K_X"])
            current_gains["K_X_D"]   = self.inputs["K_X_D"].get_float(DEFAULT_GAINS["K_X_D"])
            current_gains["K_X_INT"] = 0.0

        self.save_persistent_config()

        if self.serial and getattr(self.serial, "is_open", False):
            for attempt in range(3):
                try:
                    send_gains(
                        self.serial,
                        current_gains["K_TH"],
                        current_gains["K_TH_D"],
                        current_gains["K_X"],
                        current_gains["K_X_D"],
                        current_gains["K_X_INT"],
                        seq=attempt
                    )
                except Exception as e:
                    print(f"[TX Error] Gains send failed: {e}")
                time.sleep(0.05)
            self.gains_sent = True
            with state_lock:
                pendulum_state["gains_ack"] = False
            if not silent:
                print("Gains sent (3 packets)")
        else:
            if not silent:
                print("[Warning] Serial tidak terhubung — gains disimpan secara lokal.")

    def send_physics_params(self, silent=False):
        """Kirim parameter fisik pendulum (panjang & massa) ke STM32."""
        length_m = self.inputs_physics["length_m"].get_float(DEFAULT_PHYSICS["length_m"])
        mass_kg  = self.inputs_physics["mass_kg"].get_float(DEFAULT_PHYSICS["mass_kg"])

        # Validasi sederhana
        if not (0.01 <= length_m <= 2.0):
            if not silent:
                print(f"[ERROR] Panjang tidak valid: {length_m} m (harus 0.01–2.0 m)")
            return
        if not (0.01 <= mass_kg <= 10.0):
            if not silent:
                print(f"[ERROR] Massa tidak valid: {mass_kg} kg (harus 0.01–10.0 kg)")
            return

        self.save_persistent_config()

        if self.serial and getattr(self.serial, "is_open", False):
            for attempt in range(3):
                try:
                    send_physics(self.serial, length_m, mass_kg, seq=attempt)
                except Exception as e:
                    print(f"[TX Error] Physics send failed: {e}")
                time.sleep(0.05)
            with state_lock:
                pendulum_state["physics_sent"] = True
            if not silent:
                print(f"[Physics] Terkirim: l={length_m:.4f} m, m={mass_kg:.4f} kg")
        else:
            if not silent:
                print("[Warning] Serial tidak terhubung — parameter fisik disimpan secara lokal.")

    def start_system(self):
        if not self.gains_sent:
            # Auto-apply jika belum dikirim
            self.apply_gains(silent=True)

        with state_lock:
            pendulum_state["running"] = True
        for inp in self.inputs.values():
            inp.enabled = False
        for inp in self.inputs_physics.values():
            inp.enabled = False
        for inp in self.inputs_swing.values():
            inp.enabled = False
        self.btn_apply.enabled = False
        self.btn_send_physics.enabled = False
        self.btn_send_swing.enabled = False

        # Aktifkan metrik otomatis saat sistem dijalankan
        self.metrics.reset()
        
        # Mulai sesi logging baru
        l_val = self.inputs_physics["length_m"].get_float(DEFAULT_PHYSICS["length_m"])
        m_val = self.inputs_physics["mass_kg"].get_float(DEFAULT_PHYSICS["mass_kg"])
        with gains_lock:
            g_copy = current_gains.copy()
        self.session_logger.start_session(l_val, m_val, g_copy)
        self.session_notice = "Sesi baru dimulai..."
        self.session_notice_time = time.time()

        print("[Metrics] Metrik diaktifkan saat START")
        print("System STARTED")

    def save_session_manual(self):
        res = self.session_logger.save_session(self.metrics, reason="MANUAL_SAVE")
        if res:
            sess_id, path = res
            self.session_notice = f"✓ Sesi #{sess_id} tersimpan di Excel/CSV!"
            self.session_notice_time = time.time()
            print(f"[SessionLogger] Sesi #{sess_id} berhasil disimpan manual ke {path}")
        else:
            self.session_notice = "Tidak ada sesi aktif atau sudah tersimpan"
            self.session_notice_time = time.time()

    def reset_system(self):
        print("=" * 50)
        print("USER CLICKED RESET BUTTON")
        print("=" * 50)

        # Simpan sesi perbandingan otomatis ke CSV saat RESET
        res = self.session_logger.save_session(self.metrics, reason="RESET")
        if res:
            sess_id, path = res
            self.session_notice = f"✓ Sesi #{sess_id} tersimpan di Excel/CSV!"
            self.session_notice_time = time.time()
            print(f"[SessionLogger] Sesi #{sess_id} otomatis tersimpan ke {path}")

        if self.serial and getattr(self.serial, "is_open", False):
            print("Sending reset packet (type 0x03)...")
            try:
                send_reset(self.serial)
            except Exception as e:
                print(f"[TX Error] Reset packet send failed: {e}")
            with state_lock:
                pendulum_state["reset_ack"] = False
        else:
            print("[Warning] Serial tidak terhubung — Reset sistem secara lokal.")

        with state_lock:
            pendulum_state["running"]  = False
            pendulum_state["cmX"]      = X_CENTER_CM
            pendulum_state["theta"]    = math.pi   # Posisi gantung bawah (180°)
            pendulum_state["degree"]   = 180.0
            pendulum_state["setspeed"] = 0.0
            pendulum_state["rpm"]      = 0.0

        for inp in self.inputs.values():
            inp.enabled = True
        for inp in self.inputs_physics.values():
            inp.enabled = True
        for inp in self.inputs_swing.values():
            inp.enabled = True
        self.btn_apply.enabled = True
        self.btn_send_physics.enabled = True
        self.btn_send_swing.enabled = True

        self.metrics.deactivate()
        print("Waiting for STM32 reset ACK (0xAA 0xEE)...")

    # ─────────────────────────────────────────────────────────
    #  DRAW
    # ─────────────────────────────────────────────────────────
    def draw(self):
        self.screen.fill(C_BG)

        with state_lock:
            is_running = pendulum_state["running"]

        # ── Status indicator (kiri atas) ──
        if not getattr(self, 'serial_connected', False):
            status_text  = "OFFLINE"
            status_color = (255, 165, 0)
        elif is_running:
            status_text  = "RUNNING"
            status_color = C_STATUS_RUN
        else:
            status_text  = "STOPPED"
            status_color = C_STATUS_STOP

        pygame.draw.circle(self.screen, status_color, (22, 22), 10)
        self.screen.blit(
            self.font_large.render(status_text, True, C_TEXT), (40, 10)
        )

        # ── Top-bar buttons & Port input ──
        self.input_port.draw(self.screen, self.font_tiny, self.font_input)
        self.btn_reconnect.draw(self.screen, self.font_small)
        self.btn_rec_mode.draw(self.screen, self.font_med)
        self.btn_record.draw(self.screen,   self.font_med)
        self.btn_udp.draw(self.screen,      self.font_med)

        # ── Pendulum visualization ──
        self.draw_pendulum()

        # ── Manual controls ──
        with manual_lock:
            held_dir = manual_state["held_direction"]
            held_cmd = manual_state["held_cmd_key"]
        draw_manual_controls(self.screen, self.font_med, held_dir, held_cmd)

        # ── Panel kanan ──
        pygame.draw.rect(self.screen, C_PANEL,
                         (self.main_width, 0, self.panel_width, self.height))
        # garis pemisah vertikal
        pygame.draw.line(self.screen, C_CHART_BORDER,
                         (self.main_width, 0), (self.main_width, self.height), 1)

        # ── Tab Navigation ──
        self.btn_tab_lqr.custom_color   = (45, 110, 160) if self.active_tab == "lqr" else (40, 44, 62)
        self.btn_tab_swing.custom_color = (45, 110, 160) if self.active_tab == "swing" else (40, 44, 62)
        self.btn_tab_lqr.draw(self.screen, self.font_small)
        self.btn_tab_swing.draw(self.screen, self.font_small)

        if self.active_tab == "lqr":
            for inp in self.inputs.values():
                inp.draw(self.screen, self.font_small, self.font_input)
            self.btn_apply.draw(self.screen, self.font_med)

            div_y = self.btn_apply.rect.bottom + 8
            pygame.draw.line(self.screen, C_CHART_BORDER,
                             (self.main_width + 10, div_y),
                             (self.main_width + self.panel_width - 10, div_y), 1)
            for inp in self.inputs_physics.values():
                inp.draw(self.screen, self.font_small, self.font_input)
            self.btn_send_physics.draw(self.screen, self.font_med)

        elif self.active_tab == "swing":
            for inp in self.inputs_swing.values():
                inp.draw(self.screen, self.font_small, self.font_input)
            self.btn_send_swing.draw(self.screen, self.font_med)

            # Quick Tips Box
            tips_y = self.btn_send_swing.rect.bottom + 8
            pygame.draw.rect(self.screen, (24, 28, 42),
                             (self.main_width + 12, tips_y, self.panel_width - 24, 82),
                             border_radius=6)
            pygame.draw.rect(self.screen, (45, 55, 80),
                             (self.main_width + 12, tips_y, self.panel_width - 24, 82),
                             width=1, border_radius=6)
            t0 = self.font_tiny.render("Tips Swing-Up & Handover:", True, (100, 200, 255))
            t1 = self.font_tiny.render("• kE 70-95: Pompa energi ayunan", True, (170, 180, 205))
            t2 = self.font_tiny.render("• kx 0.4-0.7: Kereta tetap di tengah", True, (170, 180, 205))
            t3 = self.font_tiny.render("• Handover: |deg|<15°, |vel|<4 rad/s", True, (170, 180, 205))
            self.screen.blit(t0, (self.main_width + 18, tips_y + 6))
            self.screen.blit(t1, (self.main_width + 18, tips_y + 24))
            self.screen.blit(t2, (self.main_width + 18, tips_y + 42))
            self.screen.blit(t3, (self.main_width + 18, tips_y + 60))

        # ── Divider sebelum START/RESET ──
        div2_y = 462
        pygame.draw.line(self.screen, C_CHART_BORDER,
                         (self.main_width + 10, div2_y),
                         (self.main_width + self.panel_width - 10, div2_y), 1)

        self.btn_start.draw(self.screen, self.font_med)
        self.btn_save.draw(self.screen, self.font_med)
        self.btn_reset.draw(self.screen, self.font_med)

        # ── Status acks & Notifikasi Sesi ──
        with state_lock:
            gains_ack     = pendulum_state["gains_ack"]
            reset_ack     = pendulum_state["reset_ack"]
            physics_sent  = pendulum_state["physics_sent"]
            swing_ack     = pendulum_state.get("swing_ack", False)
            swing_sent    = pendulum_state.get("swing_sent", False)

        ack_y = self.btn_start.rect.bottom + 10

        if self.swing_sent and swing_ack:
            if time.time() - self.swing_ack_time > 3.0:
                self.swing_sent = False
                with state_lock:
                    pendulum_state["swing_ack"] = False

        if self.swing_sent:
            ack_text  = "✓ Swing Applied" if swing_ack else "✓ Swing Sent (Waiting ACK...)"
            ack_color = C_STATUS_RUN if swing_ack else (200, 150, 50)
            self.screen.blit(
                self.font_small.render(ack_text, True, ack_color),
                (self.main_width + 15, ack_y)
            )
            ack_y += 20

        # Notifikasi Sesi Log (muncul selama 4 detik setelah event)
        if self.session_notice and (time.time() - self.session_notice_time < 4.0):
            self.screen.blit(
                self.font_tiny.render(self.session_notice, True, (100, 220, 255)),
                (self.main_width + 15, ack_y)
            )
            ack_y += 20

        if self.gains_sent and gains_ack:
            if time.time() - self.gains_ack_time > 3.0:
                self.gains_sent = False
                with state_lock:
                    pendulum_state["gains_ack"] = False

        if self.gains_sent:
            ack_text  = "✓ Gains Applied" if gains_ack else "Waiting Gains ACK..."
            ack_color = C_STATUS_RUN if gains_ack else (200, 150, 50)
            self.screen.blit(
                self.font_small.render(ack_text, True, ack_color),
                (self.main_width + 15, ack_y)
            )
            ack_y += 22

        if not is_running and reset_ack:
            self.screen.blit(
                self.font_small.render("✓ System Ready", True, C_STATUS_RUN),
                (self.main_width + 15, ack_y)
            )
            ack_y += 22

        if physics_sent:
            self.screen.blit(
                self.font_small.render("✓ Physics Sent", True, C_UDP_ON),
                (self.main_width + 15, ack_y)
            )

        # ── Status Bar Bawah ──
        status_bar_rect = pygame.Rect(0, self.height - STATUS_BAR_HEIGHT, self.main_width, STATUS_BAR_HEIGHT)
        pygame.draw.rect(self.screen, (14, 15, 24), status_bar_rect)
        pygame.draw.line(self.screen, C_CHART_BORDER, (0, self.height - STATUS_BAR_HEIGHT), (self.main_width, self.height - STATUS_BAR_HEIGHT), 1)

        with state_lock:
            cmX   = pendulum_state["cmX"]
            theta = pendulum_state["theta"]
        theta_deg = math.degrees(theta)

        pos_text = f"Posisi: {cmX:.1f} cm   |   Sudut: {theta_deg:.2f}°"
        self.screen.blit(
            self.font_small.render(pos_text, True, C_TEXT),
            (20, self.height - STATUS_BAR_HEIGHT + 8)
        )

        with self.tx_debug_lock:
            tx_count   = self.tx_debug["count"]
            tx_ax      = self.tx_debug["last_ax"]
            tx_buttons = self.tx_debug["last_buttons"]
        tx_text = f"TX #{tx_count}   ax={tx_ax:6d}   buttons=0x{tx_buttons:02X}"
        tx_surf = self.font_tiny.render(tx_text, True, (120, 200, 130))
        self.screen.blit(
            tx_surf,
            (self.main_width - tx_surf.get_width() - 20, self.height - STATUS_BAR_HEIGHT + 10)
        )

        pygame.display.flip()

    # ─────────────────────────────────────────────────────────
    #  PENDULUM VISUALISATION
    # ─────────────────────────────────────────────────────────
    def draw_pendulum(self):
        """Gambar rel, cart, dan batang pendulum di area visualisasi."""
        # Posisi rel ditengahkan di area visualisasi
        ground_y = int((self.height - STATUS_BAR_HEIGHT) * 0.56)
        left_m   = 70
        right_m  = self.main_width - 70

        # Rel
        pygame.draw.line(self.screen, C_RAIL,
                         (left_m, ground_y), (right_m, ground_y), 4)
        for x_end in (left_m, right_m):
            pygame.draw.line(self.screen, C_RAIL,
                             (x_end, ground_y - 12), (x_end, ground_y + 12), 3)

        # Label batas rel (5 cm dan 75 cm)
        for label, x in ((f"{X_MIN_CM:.0f} cm", left_m), (f"{X_MAX_CM:.0f} cm", right_m)):
            surf = self.font_small.render(label, True, C_TEXT_DIM)
            self.screen.blit(surf, (x - surf.get_width() // 2, ground_y + 16))

        with state_lock:
            cmX   = pendulum_state["cmX"]
            theta = pendulum_state["theta"]

        alpha  = (cmX - X_MIN_CM) / (X_MAX_CM - X_MIN_CM)
        alpha  = max(0.0, min(1.0, alpha))
        cart_x = int(left_m + alpha * (right_m - left_m))
        cart_y = ground_y

        # Cart
        cw, ch = 80, 28
        cart_rect = pygame.Rect(cart_x - cw // 2, cart_y - ch // 2, cw, ch)
        pygame.draw.rect(self.screen, C_CART, cart_rect, border_radius=6)

        # Shadow/highlight cart
        pygame.draw.rect(self.screen, (140, 200, 255),
                         pygame.Rect(cart_x - cw//2 + 4, cart_y - ch//2 + 3, cw - 8, 6),
                         border_radius=3)

        # Panjang batang visual: proporsional dengan l_p GUI
        try:
            l_gui = self.inputs_physics["length_m"].get_float(DEFAULT_PHYSICS["length_m"])
        except Exception:
            l_gui = DEFAULT_PHYSICS["length_m"]
        pend_px = int(160 * (l_gui / 0.5))   # 160 px untuk 0.5 m
        pend_px = max(60, min(220, pend_px))

        pend_x = int(cart_x + pend_px * math.sin(theta))
        pend_y = int(cart_y - pend_px * math.cos(theta))

        pygame.draw.line(self.screen, C_PENDULUM,
                         (cart_x, cart_y), (pend_x, pend_y), 4)
        pygame.draw.circle(self.screen, C_MASS, (pend_x, pend_y), 12)

        # Metrics info di pojok kanan atas visualisasi
        self._draw_metrics_overlay()

    def _draw_metrics_overlay(self):
        """Tampilkan nilai metrik dan kecepatan motor di pojok kanan atas area visualisasi."""
        rt, st, os_ = self.metrics.get_current()

        with state_lock:
            rpm_val  = pendulum_state["rpm"]
            setspeed = pendulum_state["setspeed"]

        def fmt(v):
            import math as _m
            return f"{v:.3f}" if (v is not None and not _m.isnan(v)) else "—"

        # Warna RPM sesuai level
        if rpm_val < RPM_MAX * 0.33:
            rpm_color = C_RPM_LOW
        elif rpm_val < RPM_MAX * 0.66:
            rpm_color = C_RPM_MED
        else:
            rpm_color = C_RPM_HIGH

        dir_str = "FWD" if setspeed >= 0 else "REV"
        rpm_label = f"Motor Speed  : {rpm_val:.0f} RPM ({dir_str})"

        lines = [
            (rpm_label,                        rpm_color),
            (f"Rise Time    : {fmt(rt)} s",   C_RISE),
            (f"Settling Time: {fmt(st)} s",   C_SETTLING),
            (f"Overshoot    : {fmt(os_)}°",   C_OVERSHOOT),
        ]

        box_w, box_h = 265, len(lines) * 22 + 12
        ox = self.main_width - box_w - 15
        oy = 55

        # Semi-transparent background box
        box_surf = pygame.Surface((box_w, box_h), pygame.SRCALPHA)
        box_surf.fill((10, 12, 25, 190))
        self.screen.blit(box_surf, (ox, oy))
        pygame.draw.rect(self.screen, C_CHART_BORDER,
                         pygame.Rect(ox, oy, box_w, box_h), 1, border_radius=6)

        for i, (text, color) in enumerate(lines):
            surf = self.font_small.render(text, True, color)
            self.screen.blit(surf, (ox + 10, oy + 8 + i * 22))

    # ─────────────────────────────────────────────────────────
    #  MAIN LOOP
    # ─────────────────────────────────────────────────────────
    def run(self):
        self.setup_serial()

        while self.running:
            self.handle_events()
            self.draw()
            self.clock.tick(FPS)

        if self.data_logger.is_recording():
            self.data_logger.set_recording(False)
            time.sleep(0.2)

        if self.serial:
            self.serial.close()
        self.udp_broadcaster.close()
        pygame.quit()
        sys.exit(0)


def main():
    app = PendulumMonitor()
    app.run()


if __name__ == "__main__":
    main()