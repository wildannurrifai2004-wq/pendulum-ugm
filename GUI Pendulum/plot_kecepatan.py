# -*- coding: utf-8 -*-
"""
Script Generator Grafik Kecepatan Terhadap Waktu:
  1. Kecepatan Sudut Pendulum: omega(t) = theta_dot [rad/s]
  2. Kecepatan Linier Kereta:  v(t) = dx/dt [cm/s]
  3. Kecepatan Rotasi Motor:   N(t) [RPM]

Penggunaan:
  python plot_kecepatan.py                     # Otomatis memilih log pengujian terbaik
  python plot_kecepatan.py log_20260915_191636.csv  # Menentukan file log tertentu
"""

import os
import sys
import glob
import csv
import numpy as np

# Atur direktori cache matplotlib lokal agar tidak muncul warning
os.environ["MPLCONFIGDIR"] = os.path.join(os.path.dirname(__file__), ".mpl_cache")
import matplotlib.pyplot as plt

def get_target_csv(log_dir):
    # Jika pengguna memberikan argumen nama file di command line
    if len(sys.argv) > 1:
        custom_file = sys.argv[1]
        if os.path.isabs(custom_file) and os.path.isfile(custom_file):
            return custom_file
        local_path = os.path.join(log_dir, custom_file)
        if os.path.isfile(local_path):
            return local_path
        if os.path.isfile(custom_file):
            return os.path.abspath(custom_file)
        print(f"[Warning] File {custom_file} tidak ditemukan. Memilih log otomatis...")

    # Cari di folder live-tuning terlebih dahulu, lalu folder pendulum-ugm standar
    candidate_dirs = [
        os.path.join(os.path.dirname(__file__), "pendulum-ugm-live-tuning", "GUI Pendulum", "logs"),
        os.path.join(os.path.dirname(__file__), "pendulum-ugm", "GUI Pendulum", "logs"),
        os.path.join(os.path.dirname(__file__), "logs"),
        os.path.join(os.path.dirname(__file__), "GUI Pendulum", "logs")
    ]
    csv_files = []
    for d in candidate_dirs:
        if os.path.isdir(d):
            found = glob.glob(os.path.join(d, "log_*.csv"))
            if found:
                csv_files.extend(found)

    if not csv_files:
        return None

    # Urutkan berdasarkan ukuran file terbesar
    return max(csv_files, key=os.path.getsize)

def plot_velocity_graphs():
    log_dir = os.path.join(os.path.dirname(__file__), "pendulum-ugm", "GUI Pendulum", "logs")
    csv_path = get_target_csv(log_dir)

    if not csv_path:
        print("[Error] Tidak ditemukan file log_*.csv.")
        return

    csv_name = os.path.basename(csv_path)
    print(f"[Info] Membaca data pengujian dari: {csv_name}")

    t_list, cmx_list, rpm_list, omega_list = [], [], [], []
    last_t = -1.0

    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                cur_t = float(row["elapsed_s"])
                # Abaikan timestamp ganda saat flush
                if cur_t > last_t:
                    t_list.append(cur_t)
                    cmx_list.append(float(row["cmX"]))
                    rpm_list.append(float(row["motor_rpm"]))
                    # Kolom reserved2 = theta_dot_rad [rad/s]
                    omega_list.append(float(row.get("reserved2", 0.0)))
                    last_t = cur_t
            except (ValueError, KeyError):
                continue

    if len(t_list) < 5:
        print("[Error] Data di dalam log terlalu sedikit.")
        return

    t = np.array(t_list)
    cmx = np.array(cmx_list)
    rpm = np.array(rpm_list)
    omega = np.array(omega_list)

    # Kecepatan linier kereta v(t) = dx/dt dalam cm/s
    v_cart = np.gradient(cmx, t)

    # Setup Gambar 3 Subplot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
    fig.patch.set_facecolor('#FFFFFF')

    # 1. Subplot Kecepatan Sudut Pendulum omega(t)
    ax1.plot(t, omega, color="#C0392B", linewidth=1.6, label=r"$\omega(t) = \dot{\theta}(t)$")
    ax1.axhline(0, color="gray", linestyle="--", alpha=0.7, linewidth=0.8)
    ax1.set_ylabel(r"$\omega$ (rad/s)", fontsize=11, fontweight="bold")
    ax1.set_title(f"Grafik Respon Kecepatan Terhadap Waktu — [{csv_name}]", fontsize=13, fontweight="bold", pad=12)
    ax1.grid(True, linestyle=":", alpha=0.7)
    ax1.legend(loc="upper right", framealpha=0.9)

    # 2. Subplot Kecepatan Linier Kereta v(t)
    ax2.plot(t, v_cart, color="#2980B9", linewidth=1.6, label=r"$v(t) = \dot{x}(t)$ (cm/s)")
    ax2.axhline(0, color="gray", linestyle="--", alpha=0.7, linewidth=0.8)
    ax2.set_ylabel(r"$v$ (cm/s)", fontsize=11, fontweight="bold")
    ax2.grid(True, linestyle=":", alpha=0.7)
    ax2.legend(loc="upper right", framealpha=0.9)

    # 3. Subplot Kecepatan Putar Motor N(t) (RPM)
    ax3.plot(t, rpm, color="#27AE60", linewidth=1.6, label=r"Kecepatan Motor (RPM)")
    ax3.axhline(0, color="gray", linestyle="--", alpha=0.7, linewidth=0.8)
    ax3.set_xlabel("Waktu $t$ (detik)", fontsize=11, fontweight="bold")
    ax3.set_ylabel("RPM", fontsize=11, fontweight="bold")
    ax3.grid(True, linestyle=":", alpha=0.7)
    ax3.legend(loc="upper right", framealpha=0.9)

    plt.tight_layout()

    out_file = os.path.join(os.path.dirname(__file__), f"Grafik_Kecepatan_{os.path.splitext(csv_name)[0]}.png")
    plt.savefig(out_file, dpi=300, bbox_inches='tight')
    print(f"[Sukses] File grafik resolusi tinggi (300 DPI) telah disimpan di:\n  -> {out_file}")
    plt.close()

if __name__ == "__main__":
    plot_velocity_graphs()
