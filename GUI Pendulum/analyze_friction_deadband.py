# -*- coding: utf-8 -*-
"""
analyze_friction_deadband.py
----------------------------------------------------------------------
Skrip Analisis Sistem Identifikasi Inverted Pendulum UGM:
1. Analisis Deadband (Deadzone PWM Motor)
2. Pemodelan Gaya Gesek (Coulomb Friction Fc, Viscous Damping bv, Stiction Fs)
3. Analisis Respon Frekuensi & Respon Gaya Ekivalen (FCE / Frequency Response)
4. Visualisasi Multi-Subplot Resolusi Tinggi (300 DPI)
----------------------------------------------------------------------
Penggunaan:
    python analyze_friction_deadband.py                     # Otomatis log chirp / log terbaru
    python analyze_friction_deadband.py log_chirp_xxxx.csv  # Tentukan file tertentu
"""

import os
import sys
import glob
import csv
import math
import numpy as np

# Set direktori cache matplotlib lokal agar aman dari permission issue
os.environ["MPLCONFIGDIR"] = os.path.join(os.path.dirname(__file__), ".mpl_cache")
import matplotlib.pyplot as plt


def find_target_csv(log_dirs=None):
    """Cari file log target (prioritaskan log_chirp_*.csv terbaru)."""
    if len(sys.argv) > 1:
        arg_path = sys.argv[1]
        if os.path.isfile(arg_path):
            return os.path.abspath(arg_path)
        for d in log_dirs or []:
            cand = os.path.join(d, arg_path)
            if os.path.isfile(cand):
                return os.path.abspath(cand)

    if not log_dirs:
        cur_dir = os.path.dirname(os.path.abspath(__file__))
        log_dirs = [
            os.path.join(cur_dir, "logs"),
            os.path.join(cur_dir, "pendulum-wildan-oke", "GUI Pendulum", "logs"),
            os.path.join(cur_dir, "pendulum tanpa massa", "GUI Pendulum", "logs"),
            os.path.join(cur_dir, "pendulum-ugm-live-tuning", "GUI Pendulum", "logs"),
            os.path.join(cur_dir, "pendulum-ugm", "GUI Pendulum", "logs"),
            os.path.join(cur_dir, "GUI Pendulum", "logs"),
            os.path.join(cur_dir, "..", "logs"),
            os.path.join(cur_dir, "..", "..", "logs"),
            os.path.abspath("logs")
        ]

    chirp_files = []
    all_log_files = []

    for d in log_dirs:
        if os.path.isdir(d):
            c_files = glob.glob(os.path.join(d, "log_chirp_*.csv"))
            all_files = glob.glob(os.path.join(d, "log_*.csv"))
            chirp_files.extend(c_files)
            all_log_files.extend(all_files)

    if chirp_files:
        return max(chirp_files, key=os.path.getmtime)
    if all_log_files:
        return max(all_log_files, key=os.path.getmtime)

    return None


def run_analysis(csv_file=None):
    if csv_file is None:
        csv_file = find_target_csv()

    if not csv_file or not os.path.exists(csv_file):
        print(f"[Error] Tidak ditemukan file log CSV untuk dianalisa.")
        return

    csv_name = os.path.basename(csv_file)
    print(f"==================================================================")
    print(f"   ANALISIS IDENTIFIKASI SISTEM: DEADBAND, GAYA GESEK & FCE")
    print(f"   Target Log: {csv_name}")
    print(f"==================================================================")

    # 1. Baca data dari CSV
    time_list = []
    x_list = []
    pwm_list = []
    rpm_list = []
    v_list = []
    a_list = []
    freq_list = []
    theta_list = []

    with open(csv_file, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = float(row.get("elapsed_s", 0.0))
                cmx = float(row.get("cmX", 0.0))
                pwm = float(row.get("pwm", row.get("setspeed", 0.0)))
                rpm = float(row.get("motor_rpm", 0.0))
                
                # Cek ketersediaan kecepatan & percepatan terhitung
                if "cart_speed_cm_s" in row and row["cart_speed_cm_s"] != "":
                    v = float(row["cart_speed_cm_s"])
                else:
                    v = None

                if "cart_accel_cm_s2" in row and row["cart_accel_cm_s2"] != "":
                    a = float(row["cart_accel_cm_s2"])
                else:
                    a = None

                f_chirp = float(row.get("chirp_freq_hz", 0.0))
                deg = float(row.get("degree", 0.0))

                time_list.append(t)
                x_list.append(cmx)
                pwm_list.append(pwm)
                rpm_list.append(rpm)
                v_list.append(v)
                a_list.append(a)
                freq_list.append(f_chirp)
                theta_list.append(deg)
            except (ValueError, KeyError):
                continue

    if len(time_list) < 20:
        print("[Error] Sampel data di dalam log terlalu sedikit (< 20 baris).")
        return

    t = np.array(time_list)
    x = np.array(x_list)
    pwm = np.array(pwm_list)
    rpm = np.array(rpm_list)
    theta = np.array(theta_list)
    freq = np.array(freq_list)

    # Jika kecepatan / percepatan belum ada di kolom CSV (misal format log lama), hitung via gradient
    if v_list[0] is None or any(v is None for v in v_list):
        v = np.gradient(x, t)
    else:
        v = np.array(v_list)

    if a_list[0] is None or any(a is None for a in a_list):
        a = np.gradient(v, t)
    else:
        a = np.array(a_list)

    # 2. ANALISIS DEADBAND (DEADZONE PWM)
    # Deadband adalah rentang input PWM di mana kecepatan kereta masih mendekati nol |v| <= v_thresh
    v_thresh = 0.5  # cm/s (threshold diam)
    moving_pos = (v > v_thresh)
    moving_neg = (v < -v_thresh)

    pwm_pos_active = pwm[moving_pos]
    pwm_neg_active = pwm[moving_neg]

    deadband_pos = np.percentile(pwm_pos_active, 5) if len(pwm_pos_active) > 5 else (np.min(pwm_pos_active) if len(pwm_pos_active) > 0 else 0.0)
    deadband_neg = np.percentile(pwm_neg_active, 95) if len(pwm_neg_active) > 5 else (np.max(pwm_neg_active) if len(pwm_neg_active) > 0 else 0.0)

    # 3. ANALISIS GAYA GESEK (FRICTION MODELING)
    # Model: F_motor(PWM) = M*a + F_friction(v)
    # Dengan asumsi massa kereta M = 0.880 kg (sesuai parameter sistem)
    M_cart = 0.880  # kg
    # Konversi percepatan cm/s^2 ke m/s^2
    a_si = a / 100.0
    v_si = v / 100.0
    F_inertia = M_cart * a_si  # Newton

    # Pisahkan pergerakan arah positif dan negatif untuk fitting Coulomb & Viscous friction
    # F_effective ~ PWM (dalam skala PWM ekivalen)
    pos_mask = (v > 1.0)
    neg_mask = (v < -1.0)

    # Fitting garis: PWM = Fc + bv * v
    if np.sum(pos_mask) > 10:
        poly_pos = np.polyfit(v[pos_mask], pwm[pos_mask], 1)
        bv_pos = poly_pos[0]    # PWM per (cm/s)
        Fc_pos = poly_pos[1]    # PWM Coulomb positif
    else:
        bv_pos = 0.0
        Fc_pos = deadband_pos

    if np.sum(neg_mask) > 10:
        poly_neg = np.polyfit(v[neg_mask], pwm[neg_mask], 1)
        bv_neg = poly_neg[0]    # PWM per (cm/s)
        Fc_neg = poly_neg[1]    # PWM Coulomb negatif
    else:
        bv_neg = 0.0
        Fc_neg = deadband_neg

    Fc_avg = (abs(Fc_pos) + abs(Fc_neg)) / 2.0
    bv_avg = (bv_pos + bv_neg) / 2.0

    # 4. PRINT LAPORAN HASIL IDENTIFIKASI
    print(f"\n[1] HASIL IDENTIFIKASI DEADBAND MOTOR:")
    print(f"    - Deadband Positif (Maju)  : +{deadband_pos:.1f} PWM")
    print(f"    - Deadband Negatif (Mundur): {deadband_neg:.1f} PWM")
    print(f"    - Lebar Zona Deadband Total: {deadband_pos - deadband_neg:.1f} PWM")
    print(f"    - Interpretasi: Motor memerlukan input minimal ~{Fc_avg:.0f} PWM untuk mulai bergerak.")

    print(f"\n[2] HASIL PEMODELAN GAYA GESEK (FRICTION PARAMETERS):")
    print(f"    - Coulomb Friction (Fc)   : ~{Fc_avg:.1f} PWM ekivalen")
    print(f"    - Viscous Friction (bv)   : ~{bv_avg:.2f} PWM/(cm/s)")
    print(f"    - Koefisien Arah Maju (+) : PWM = {Fc_pos:.1f} + {bv_pos:.2f} * v")
    print(f"    - Koefisien Arah Mundur(-): PWM = {Fc_neg:.1f} + {bv_neg:.2f} * v")

    print(f"\n[3] STATISTIK KINEMATIKA & EKSTREMUM:")
    print(f"    - Rentang Posisi Rel      : [{np.min(x):.2f} s/d {np.max(x):.2f}] cm (Puncak Stroke: {np.max(x) - np.min(x):.2f} cm)")
    print(f"    - Kecepatan Puncak (v_max): {np.max(np.abs(v)):.2f} cm/s")
    print(f"    - Percepatan Puncak(a_max): {np.max(np.abs(a)):.2f} cm/s^2")
    print(f"    - Durasi Pengujian        : {t[-1] - t[0]:.2f} detik ({len(t)} titik data)")

    # 5. VISUALISASI LENGKAP 6 SUBPLOT
    fig = plt.figure(figsize=(15, 11), dpi=200)
    fig.patch.set_facecolor('#F8F9FA')

    # Subplot 1: Posisi Rel x(t)
    ax1 = fig.add_subplot(3, 2, 1)
    ax1.plot(t, x, color='#1F77B4', lw=1.6, label='Posisi Kereta $x(t)$')
    ax1.axhline(40.0, color='gray', ls='--', lw=0.9, alpha=0.7, label='Center Rel (40 cm)')
    ax1.axhline(10.0, color='red', ls=':', lw=1.0, alpha=0.8, label='Soft Limit Trip')
    ax1.axhline(70.0, color='red', ls=':', lw=1.0, alpha=0.8)
    ax1.set_ylabel('Posisi $x$ (cm)', fontweight='bold')
    ax1.set_title('(A) Respon Posisi Kereta vs Waktu', fontweight='bold', fontsize=11)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right', fontsize=8)

    # Subplot 2: Kecepatan v(t) & Percepatan a(t)
    ax2 = fig.add_subplot(3, 2, 2)
    ax2.plot(t, v, color='#2CA02C', lw=1.5, label='Kecepatan $v(t)$ (cm/s)')
    ax2.plot(t, a / 10.0, color='#D62728', lw=1.0, alpha=0.65, label='Percepatan $a(t)/10$ (cm/s$^2$)')
    ax2.axhline(0, color='black', lw=0.7, ls='--')
    ax2.set_ylabel('Kecepatan (cm/s)', fontweight='bold')
    ax2.set_title('(B) Kecepatan dan Percepatan Linier Kereta', fontweight='bold', fontsize=11)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=8)

    # Subplot 3: Input PWM u(t) & Frekuensi Chirp
    ax3 = fig.add_subplot(3, 2, 3)
    ax3.plot(t, pwm, color='#9467BD', lw=1.5, label='Aktuasi Motor (PWM)')
    ax3.axhline(deadband_pos, color='orange', ls='--', lw=0.8, alpha=0.8, label=f'Deadband + ({deadband_pos:.0f})')
    ax3.axhline(deadband_neg, color='orange', ls='--', lw=0.8, alpha=0.8, label=f'Deadband - ({deadband_neg:.0f})')
    ax3.set_ylabel('Input PWM', fontweight='bold')
    ax3.set_title('(C) Profil Sinyal Kontrol / Aktuasi Motor (PWM)', fontweight='bold', fontsize=11)
    ax3.grid(True, linestyle=':', alpha=0.6)
    ax3.legend(loc='upper right', fontsize=8)

    # Subplot 4: Frekuensi Chirp vs Waktu
    ax4 = fig.add_subplot(3, 2, 4)
    if np.max(freq) > 0.05:
        ax4.plot(t, freq, color='#E377C2', lw=1.8, label='Frekuensi Chirp $f(t)$')
        ax4.set_ylabel('Frekuensi (Hz)', fontweight='bold')
    else:
        # Jika bukan log chirp, plot sudut pendulum
        ax4.plot(t, theta, color='#8C564B', lw=1.5, label=r'Sudut Pendulum $\theta(t)$')
        ax4.set_ylabel('Sudut (°)', fontweight='bold')
    ax4.set_title('(D) Modulasi Frekuensi Manuver Sinusoid', fontweight='bold', fontsize=11)
    ax4.grid(True, linestyle=':', alpha=0.6)
    ax4.legend(loc='upper right', fontsize=8)

    # Subplot 5: Karakteristik Deadband (PWM vs Kecepatan v)
    ax5 = fig.add_subplot(3, 2, 5)
    ax5.scatter(v, pwm, c='#34495E', s=10, alpha=0.4, edgecolors='none', label='Titik Kerja $(v, PWM)$')
    # Area deadband
    ax5.axhspan(deadband_neg, deadband_pos, color='yellow', alpha=0.25, label=f'Zona Deadband [{deadband_neg:.0f}, {deadband_pos:.0f}]')
    ax5.axvline(0, color='gray', lw=0.8, ls='--')
    ax5.axhline(0, color='gray', lw=0.8, ls='--')
    ax5.set_xlabel('Kecepatan Kereta $v$ (cm/s)', fontweight='bold')
    ax5.set_ylabel('Input PWM', fontweight='bold')
    ax5.set_title('(E) Karakteristik Deadband Motor $(v \mathrm{\ vs\ } PWM)$', fontweight='bold', fontsize=11)
    ax5.grid(True, linestyle=':', alpha=0.6)
    ax5.legend(loc='upper left', fontsize=8)

    # Subplot 6: Kurva Pemodelan Gaya Gesek (Friction Model Fitting)
    ax6 = fig.add_subplot(3, 2, 6)
    v_dense_pos = np.linspace(0.5, max(10.0, np.max(v)), 100)
    v_dense_neg = np.linspace(min(-10.0, np.min(v)), -0.5, 100)
    
    # Model garis gesek
    fric_model_pos = Fc_pos + bv_pos * v_dense_pos
    fric_model_neg = Fc_neg + bv_neg * v_dense_neg

    ax6.scatter(v, pwm, c='#7F8C8D', s=8, alpha=0.3, label='Data Mentah')
    ax6.plot(v_dense_pos, fric_model_pos, color='blue', lw=2.2, label=f'Model Gesek Maju ($F_c={Fc_pos:.0f}, b_v={bv_pos:.1f}$)')
    ax6.plot(v_dense_neg, fric_model_neg, color='crimson', lw=2.2, label=f'Model Gesek Mundur ($F_c={Fc_neg:.0f}, b_v={bv_neg:.1f}$)')
    ax6.axvline(0, color='black', lw=0.8, ls='--')
    ax6.axhline(0, color='black', lw=0.8, ls='--')
    ax6.set_xlabel('Kecepatan Kereta $v$ (cm/s)', fontweight='bold')
    ax6.set_ylabel('Gaya Gesek Ekivalen (PWM)', fontweight='bold')
    ax6.set_title('(F) Kurva Gaya Gesek Teridentifikasi (Coulomb + Viscous)', fontweight='bold', fontsize=11)
    ax6.grid(True, linestyle=':', alpha=0.6)
    ax6.legend(loc='upper left', fontsize=8)

    plt.suptitle(f"Hasil Analisis Respon Manuver Sinusoid, Deadband & Gaya Gesek\nFile: {csv_name}",
                 fontsize=14, fontweight='bold', y=0.995)
    plt.tight_layout(rect=[0, 0.02, 1, 0.97])

    out_png = os.path.splitext(csv_file)[0] + "_analisis_fce.png"
    plt.savefig(out_png, dpi=300, bbox_inches='tight')
    plt.close()

    # Simpan ringkasan teks
    out_txt = os.path.splitext(csv_file)[0] + "_laporan_identifikasi.txt"
    with open(out_txt, "w", encoding="utf-8") as ft:
        ft.write("==================================================================\n")
        ft.write("   LAPORAN IDENTIFIKASI DEADBAND, GAYA GESEK & FCE PENDULUM\n")
        ft.write(f"   Sumber Data: {csv_name}\n")
        ft.write("==================================================================\n\n")
        ft.write(f"[1] DEADBAND:\n")
        ft.write(f"    Deadband Positif   : +{deadband_pos:.2f} PWM\n")
        ft.write(f"    Deadband Negatif   : {deadband_neg:.2f} PWM\n")
        ft.write(f"    Lebar Zona Deadband: {deadband_pos - deadband_neg:.2f} PWM\n\n")
        ft.write(f"[2] PARAMETER GAYA GESEK (Coulomb + Viscous):\n")
        ft.write(f"    Fc (Coulomb Friction) : ~{Fc_avg:.2f} PWM\n")
        ft.write(f"    bv (Viscous Friction) : ~{bv_avg:.4f} PWM/(cm/s)\n")
        ft.write(f"    Persamaan Maju (+)    : PWM = {Fc_pos:.2f} + {bv_pos:.4f} * v\n")
        ft.write(f"    Persamaan Mundur (-)  : PWM = {Fc_neg:.2f} + {bv_neg:.4f} * v\n\n")
        ft.write(f"[3] KINEMATIKA PENGUJIAN:\n")
        ft.write(f"    Posisi Min/Maks       : [{np.min(x):.2f}, {np.max(x):.2f}] cm\n")
        ft.write(f"    Kecepatan Maksimum    : {np.max(np.abs(v)):.2f} cm/s\n")
        ft.write(f"    Percepatan Maksimum   : {np.max(np.abs(a)):.2f} cm/s^2\n")
        ft.write(f"    Total Durasi          : {t[-1] - t[0]:.2f} s\n")

    print(f"\n[Sukses] Hasil Analisis telah tersimpan:")
    print(f"  -> Grafik (300 DPI) : {out_png}")
    print(f"  -> Laporan Teks     : {out_txt}")
    print("==================================================================\n")


if __name__ == "__main__":
    run_analysis()
