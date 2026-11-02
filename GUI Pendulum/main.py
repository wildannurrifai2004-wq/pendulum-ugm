import os
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
os.environ["SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import sys
import threading
import time
import pygame

import math
from threading import Lock

from lib_com import open_serial, read_control_status, make_packet
from lib_gui import (
    init_display, draw_ui, is_record_button, draw_pendulum,
    draw_manual_controls, get_manual_hit, get_cmd_hit,
    MANUAL_AX_SPEED, CMD_BUTTONS,
)
from lib_data import DataLogger

PORT = "COM12"           # ganti sesuai port kamu
BAUD = 115200
FPS  = 50               # 50 Hz

pendulum_state = {
    "cmX": 0.0,
    "theta": 0.0
}
state_lock = Lock()

# === STATE KENDALI MANUAL (pengganti stick) ===
manual_state = {
    "ax": 0,          # -32767..32767, diisi tombol KIRI/KANAN
    "buttons": 0,     # 0 / 1 / 2 / 4 / 8, diisi tombol command
    "held_direction": None,   # "left" / "right" / None -> untuk highlight
    "held_cmd_key": None,     # key di CMD_BUTTONS / None -> untuk highlight
}
manual_lock = Lock()


def manual_sender(ser, fps=50):
    """Thread pengirim state tombol GUI -> STM32, format paket SAMA seperti stick (type 0x01)."""
    period = 1.0 / fps
    seq = 0
    while True:
        with manual_lock:
            ax = manual_state["ax"]
            buttons = manual_state["buttons"]

        pkt = make_packet(seq, ax, 0, 0, 0, buttons)
        ser.write(pkt)
        seq = (seq + 1) & 0xFF

        time.sleep(period)

def main():
    # === SERIAL ===
    ser = open_serial(PORT, BAUD)
    print("Serial opened:", ser.port, BAUD)

    # === DATA LOGGER ===
    data_logger = DataLogger(base_dir="logs")

    # === PYGAME ===
    pygame.init()
    screen, font, clock = init_display(600, 450, "Button Control → Serial Pendulum MIPA")

    # === THREAD TOMBOL GUI -> STM32 (format paket sama seperti stick) ===
    t_tx = threading.Thread(
        target=manual_sender,
        args=(ser, FPS),
        daemon=True
    )
    t_tx.start()

    # === THREAD STM32 -> PC (baca control_status) ===
    def on_status(sample_tuple):
        # sample_tuple: (logtick, degree, cmX, setspeed, r1, r2, r3)
        
        logtick, degree, cmX, setspeed, r1, theta_dot, theta = sample_tuple
        
        with state_lock:
            pendulum_state["cmX"] = cmX
            pendulum_state["theta"] = theta

        data_logger.handle_sample(sample_tuple)

    t_rx = threading.Thread(
        target=read_control_status,
        args=(ser, on_status),
        kwargs={"debug": False},
        daemon=True
    )
    t_rx.start()

    # === LOOP GUI UTAMA ===
    seq = 0
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if is_record_button(event.pos):
                    data_logger.toggle_recording()
                    print("Recording:", data_logger.is_recording())

                direction = get_manual_hit(event.pos)
                if direction is not None:
                    with manual_lock:
                        manual_state["ax"] = (
                            -MANUAL_AX_SPEED if direction == "left" else MANUAL_AX_SPEED
                        )
                        manual_state["held_direction"] = direction

                cmd_key = get_cmd_hit(event.pos)
                if cmd_key is not None:
                    with manual_lock:
                        manual_state["buttons"] = CMD_BUTTONS[cmd_key]["value"]
                        manual_state["held_cmd_key"] = cmd_key

            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                # lepas tombol -> balik ke netral (ax=0 / buttons=0), sama seperti lepas stick
                with manual_lock:
                    manual_state["ax"] = 0
                    manual_state["buttons"] = 0
                    manual_state["held_direction"] = None
                    manual_state["held_cmd_key"] = None

        with manual_lock:
            ax = manual_state["ax"]
            buttons = manual_state["buttons"]
            held_direction = manual_state["held_direction"]
            held_cmd_key = manual_state["held_cmd_key"]

        recording = data_logger.is_recording()

        # Gambar tampilan
        draw_ui(screen, font, ax, 0, 0, 0, buttons, seq, recording)
        draw_manual_controls(screen, font, held_direction, held_cmd_key)

        with state_lock:
            cmX = pendulum_state["cmX"]
            theta = pendulum_state["theta"]

        draw_pendulum(screen, cmX, theta)

        pygame.display.flip()
        clock.tick(FPS)
        seq = (seq + 1) & 0xFF

    # keluar
    ser.close()
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()