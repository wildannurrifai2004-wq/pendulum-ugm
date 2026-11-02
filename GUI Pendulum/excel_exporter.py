"""
excel_exporter.py - Konversi CSV Log Pendulum ke Excel (.xlsx) dengan Grafik & Metrik Otomatis

Metrik yang dihitung:
- Rise Time (s): Waktu pertama kali |theta| <= 5°
- Settling Time (s): Waktu |theta| masuk dan tetap di <= 2° selama 0.5s
- Overshoot (°): Nilai puncak maksimum |theta| dalam derajat
"""

import os
import csv
import math
import sys
from openpyxl import Workbook
from openpyxl.chart import LineChart, Reference, Series
from openpyxl.styles import Font, PatternFill, Alignment, Border, Side


def process_csv_and_export_excel(csv_path: str, excel_path: str = None, stick_length: float = None) -> str:
    """
    Membaca log CSV pendulum dan membuat file Excel (.xlsx)
    berisi ringkasan metrik, grafik otomatis, dan sheet data mentah.
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"File CSV tidak ditemukan: {csv_path}")

    if excel_path is None:
        excel_path = os.path.splitext(csv_path)[0] + ".xlsx"

    # 1. Baca data dari CSV
    rows = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for r in reader:
            if r:
                rows.append(r)

    if not rows or not header:
        print(f"File CSV kosong: {csv_path}")
        return None

    # Tentukan index kolom
    col_map = {name.lower().strip(): idx for idx, name in enumerate(header)}
    
    has_datetime = 'datetime' in col_map
    has_timestamp = 'timestamp' in col_map
    has_elapsed = 'elapsed_s' in col_map
    
    idx_degree = col_map.get('degree', 1 if not has_elapsed else col_map['degree'])
    idx_cmx = col_map.get('cmx', 2 if not has_elapsed else col_map['cmx'])
    idx_setspeed = col_map.get('motor_rpm', col_map.get('setspeed', 3 if not has_elapsed else 3))
    idx_logtick = col_map.get('logtick', 0 if not has_elapsed else col_map['logtick'])
    idx_elapsed = col_map.get('elapsed_s', None)
    idx_datetime = col_map.get('datetime', None)

    parsed_data = []
    t_start = None

    for i, r in enumerate(rows):
        try:
            logtick = int(float(r[idx_logtick])) if idx_logtick < len(r) else i
            degree = float(r[idx_degree]) if idx_degree < len(r) else 0.0
            cmX = float(r[idx_cmx]) if idx_cmx < len(r) else 0.0
            setspeed = float(r[idx_setspeed]) if idx_setspeed < len(r) else 0.0

            if idx_elapsed is not None and idx_elapsed < len(r):
                elapsed_s = float(r[idx_elapsed])
            else:
                # Estimasi waktu jika logtick ada (misal 50Hz / 20ms per tick)
                if i == 0:
                    t_start = logtick
                elapsed_s = (logtick - t_start) * 0.001 if (t_start is not None and logtick >= t_start) else i * 0.02

            dt_str = r[idx_datetime] if idx_datetime is not None and idx_datetime < len(r) else f"T+{elapsed_s:.3f}s"

            # Ambil reserved fields jika ada
            res = [float(r[j]) if j < len(r) else 0.0 for j in range(idx_setspeed + 1, len(r))]
            while len(res) < 4:
                res.append(0.0)

            parsed_data.append({
                'datetime': dt_str,
                'elapsed_s': elapsed_s,
                'logtick': logtick,
                'degree': degree,
                'cmX': cmX,
                'setspeed': setspeed,
                'res1': res[0],
                'res2': res[1],
                'res3': res[2],
                'res4': res[3],
            })
        except ValueError:
            continue

    if not parsed_data:
        print(f"Data tidak valid di CSV: {csv_path}")
        return None

    # 2. Hitung Metrik Performa (Rise Time, Settling Time, Overshoot)
    RISE_TARGET_DEG = 5.0
    SETTLE_BAND_DEG = 2.0
    SETTLE_DWELL_S = 0.5

    peak_deg = 0.0
    rise_time_s = None
    settling_time_s = None
    t_enter_band = None

    hist_rise = []
    hist_settling = []
    hist_overshoot = []

    t_balance_start = None
    balance_active = False

    for item in parsed_data:
        t = item['elapsed_s']
        deg = abs(item['degree'])

        # Titik acuan t_balance_start saat LQR Catch aktif (<= 15.0°)
        if t_balance_start is None and deg <= 15.0:
            t_balance_start = t
            balance_active = True

        t_rel = (t - t_balance_start) if t_balance_start is not None else t

        # Overshoot & Metrik (hanya dihitung selama berada di area kontrol balance <= 20.0°)
        if balance_active:
            if deg > 20.0:
                balance_active = False  # Keluar dari kontrol / pendulum jatuh ke 180°
            else:
                if deg > peak_deg:
                    peak_deg = deg

                # Rise Time (waktu dari LQR catch ke <= 5.0°)
                if rise_time_s is None:
                    if deg <= RISE_TARGET_DEG:
                        rise_time_s = t_rel

                # Settling Time (masuk <= 2.0° dan stabil selama 1.0 detik penuh)
                if settling_time_s is None:
                    if deg <= SETTLE_BAND_DEG:
                        if t_enter_band is None:
                            t_enter_band = t_rel
                        elif (t_rel - t_enter_band) >= SETTLE_DWELL_S:
                            settling_time_s = t_rel # Catat waktu setelah terverifikasi stabil 0.5s penuh
                else:
                    t_enter_band = None

        hist_rise.append(rise_time_s if rise_time_s is not None else None)
        hist_settling.append(settling_time_s if settling_time_s is not None else None)
        hist_overshoot.append(peak_deg if rise_time_s is not None else None)

    overshoot_deg = peak_deg

    # 3. Buat Workbook Excel dengan openpyxl
    wb = Workbook()

    # Styling Excel
    font_header = Font(name='Segoe UI', size=11, bold=True, color='FFFFFF')
    fill_header = PatternFill(start_color='1F497D', end_color='1F497D', fill_type='solid') # Navy
    font_title = Font(name='Segoe UI', size=16, bold=True, color='1F497D')
    font_subtitle = Font(name='Segoe UI', size=11, italic=True, color='595959')

    fill_metric_hdr = PatternFill(start_color='244062', end_color='244062', fill_type='solid')
    fill_metric_val = PatternFill(start_color='F2F5F9', end_color='F2F5F9', fill_type='solid')
    font_bold = Font(name='Segoe UI', size=11, bold=True)
    font_normal = Font(name='Segoe UI', size=11)

    thin_border = Border(
        left=Side(style='thin', color='D9D9D9'),
        right=Side(style='thin', color='D9D9D9'),
        top=Side(style='thin', color='D9D9D9'),
        bottom=Side(style='thin', color='D9D9D9')
    )

    # ---------------------------------------------------------
    # Sheet 1: Ringkasan & Grafik
    # ---------------------------------------------------------
    ws_summary = wb.active
    ws_summary.title = "Ringkasan & Grafik"
    ws_summary.views.sheetView[0].showGridLines = True

    ws_summary['B2'] = "LAPORAN METRIK PERFORMA PENDULUM"
    ws_summary['B2'].font = font_title
    ws_summary['B3'] = f"File Sumber: {os.path.basename(csv_path)} | Total Sample: {len(parsed_data)}"
    ws_summary['B3'].font = font_subtitle

    # Tabel Ringkasan Metrik dengan Nilai Angka + Satuan Langsung
    max_cmx_val = max(abs(d['cmX']) for d in parsed_data) if parsed_data else 0.0
    total_dur_val = parsed_data[-1]['elapsed_s'] if parsed_data else 0.0

    metric_headers = ["Parameter Metrik", "Nilai + Satuan", "Satuan", "Keterangan"]
    metric_data = [
        ["Panjang Batang (l_p)", f"{stick_length:.3f} m" if stick_length is not None else "N/A", "meter (m)", "Panjang pendulum dari GUI"],
        ["Rise Time (Tr)", f"{rise_time_s:.3f} detik" if rise_time_s is not None else "N/A", "detik", "Waktu pertama sudut <= 5°"],
        ["Settling Time (Ts)", f"{settling_time_s:.3f} detik" if settling_time_s is not None else "N/A", "detik", "Waktu stabil di band ±2° selama 0.5s"],
        ["Overshoot (Mp)", f"{overshoot_deg:.2f}°", "derajat (°)", "Sudut puncak maksimum"],
        ["Posisi Maks (cmX)", f"{max_cmx_val:.2f} cm", "cm", "Simpangan posisi terbesar kereta"],
        ["Durasi Eksperimen", f"{total_dur_val:.2f} detik", "detik", "Total durasi pencatatan log"]
    ]

    # Writing Metric Table
    start_row = 5
    for col_idx, text in enumerate(metric_headers, start=2):
        cell = ws_summary.cell(row=start_row, column=col_idx, value=text)
        cell.font = font_header
        cell.fill = fill_metric_hdr
        cell.alignment = Alignment(horizontal='center', vertical='center')

    for r_idx, row_vals in enumerate(metric_data, start=start_row + 1):
        for col_idx, val in enumerate(row_vals, start=2):
            cell = ws_summary.cell(row=r_idx, column=col_idx, value=val)
            cell.font = font_bold if col_idx == 3 else font_normal
            cell.fill = fill_metric_val
            cell.border = thin_border
            cell.alignment = Alignment(horizontal='center' if col_idx in [3, 4] else 'left', vertical='center')

    # Column widths for Summary Sheet
    ws_summary.column_dimensions['A'].width = 3
    ws_summary.column_dimensions['B'].width = 24
    ws_summary.column_dimensions['C'].width = 20
    ws_summary.column_dimensions['D'].width = 14
    ws_summary.column_dimensions['E'].width = 42

    # ---------------------------------------------------------
    # Sheet 2: Data Log
    # ---------------------------------------------------------
    ws_data = wb.create_sheet(title="Data Log")
    ws_data.views.sheetView[0].showGridLines = True

    data_headers = ["Waktu (DateTime)", "Waktu Relatif (detik)", "Tick STM32", "Sudut Theta (derajat °)",
                    "Posisi cmX (cm)", "Kecepatan Motor (RPM)", "Rise Time (detik)", "Settling Time (detik)", "Overshoot (derajat °)"]

    for col_idx, text in enumerate(data_headers, start=1):
        cell = ws_data.cell(row=1, column=col_idx, value=text)
        cell.font = font_header
        cell.fill = fill_header
        cell.alignment = Alignment(horizontal='center', vertical='center')

    for row_idx, (item, r_time, s_time, ov_deg) in enumerate(zip(parsed_data, hist_rise, hist_settling, hist_overshoot), start=2):
        c1 = ws_data.cell(row=row_idx, column=1, value=item['datetime'])
        
        c2 = ws_data.cell(row=row_idx, column=2, value=round(item['elapsed_s'], 3))
        c2.number_format = '0.000" s"'

        c3 = ws_data.cell(row=row_idx, column=3, value=item['logtick'])

        c4 = ws_data.cell(row=row_idx, column=4, value=round(item['degree'], 3))
        c4.number_format = '0.000"°"'

        c5 = ws_data.cell(row=row_idx, column=5, value=round(item['cmX'], 3))
        c5.number_format = '0.000" cm"'

        c6 = ws_data.cell(row=row_idx, column=6, value=round(item['setspeed'], 2))

        c7 = ws_data.cell(row=row_idx, column=7, value=round(r_time, 3) if r_time is not None else "")
        if r_time is not None:
            c7.number_format = '0.000" s"'

        c8 = ws_data.cell(row=row_idx, column=8, value=round(s_time, 3) if s_time is not None else "")
        if s_time is not None:
            c8.number_format = '0.000" s"'

        c9 = ws_data.cell(row=row_idx, column=9, value=round(ov_deg, 2) if ov_deg is not None else "")
        if ov_deg is not None:
            c9.number_format = '0.00"°"'

    # Auto-adjust column widths for Data Log
    for col in ws_data.columns:
        max_len = max(len(str(cell.value or '')) for cell in col)
        col_letter = col[0].column_letter
        ws_data.column_dimensions[col_letter].width = max(max_len + 3, 14)

    # ---------------------------------------------------------
    # MEMBUAT GRAFIK AUTOMATIS DI SHEET 1
    # ---------------------------------------------------------
    num_rows = len(parsed_data) + 1

    # 1. Grafik Respon Sudut (Theta vs Waktu)
    chart_theta = LineChart()
    chart_theta.title = "Respon Sudut Pendulum (Theta vs Waktu)"
    chart_theta.style = 13
    chart_theta.y_axis.title = "Sudut Theta (derajat °)"
    chart_theta.x_axis.title = "Waktu (detik)"
    chart_theta.width = 20
    chart_theta.height = 10

    chart_theta.y_axis.delete = False
    chart_theta.y_axis.tickLblPos = "nextTo"
    chart_theta.y_axis.majorTickMark = "out"

    chart_theta.x_axis.delete = False
    chart_theta.x_axis.tickLblPos = "nextTo"
    chart_theta.x_axis.majorTickMark = "out"

    data_theta = Reference(ws_data, min_col=4, min_row=1, max_row=num_rows)
    cats_time = Reference(ws_data, min_col=2, min_row=2, max_row=num_rows)
    chart_theta.add_data(data_theta, titles_from_data=True)
    chart_theta.set_categories(cats_time)

    chart_theta.legend = None
    chart_theta.x_axis.tickLblSkip = 50

    if chart_theta.series:
        chart_theta.series[0].graphicalProperties.line.solidFill = "1F497D" # Navy
        chart_theta.series[0].graphicalProperties.line.width = 20000

    ws_summary.add_chart(chart_theta, "B13")

    # 2. Grafik Metrik Performa (Rise Time, Settling Time, Overshoot)
    # Sumbu Y Utama = Waktu (detik), Sumbu Y Sekunder = Overshoot (derajat °)
    chart_metrics = LineChart()
    chart_metrics.title = "Pengaruh Sudut Awal terhadap Rise Time, Settling Time, dan Overshoot"
    chart_metrics.style = 10
    chart_metrics.legend.position = "t"
    chart_metrics.y_axis.title = "Waktu Respons (detik)"
    chart_metrics.x_axis.title = "Waktu (detik)"
    chart_metrics.x_axis.tickLblSkip = 50
    chart_metrics.width = 20
    chart_metrics.height = 11

    # Konfigurasi Sumbu Y Utama (Menampilkan Angka Skala 0, 1, 2, 3...)
    chart_metrics.y_axis.delete = False
    chart_metrics.y_axis.tickLblPos = "nextTo"
    chart_metrics.y_axis.majorTickMark = "out"
    chart_metrics.y_axis.number_format = '0.0'
    chart_metrics.y_axis.scaling.min = 0

    # Konfigurasi Sumbu X
    chart_metrics.x_axis.delete = False
    chart_metrics.x_axis.tickLblPos = "nextTo"
    chart_metrics.x_axis.majorTickMark = "out"

    # Data Rise Time (kolom 7, detik) dan Settling Time (kolom 8, detik)
    data_time_metrics = Reference(ws_data, min_col=7, min_row=1, max_col=8, max_row=num_rows)
    chart_metrics.add_data(data_time_metrics, titles_from_data=True)
    chart_metrics.set_categories(cats_time)

    # Custom line colors untuk Rise Time & Settling Time
    if len(chart_metrics.series) >= 1:
        chart_metrics.series[0].graphicalProperties.line.solidFill = "1F497D" # Navy (Rise Time)
        chart_metrics.series[0].graphicalProperties.line.width = 24000
        chart_metrics.series[0].marker.symbol = "circle"
        chart_metrics.series[0].marker.size = 5
    if len(chart_metrics.series) >= 2:
        chart_metrics.series[1].graphicalProperties.line.solidFill = "C00000" # Red (Settling Time)
        chart_metrics.series[1].graphicalProperties.line.width = 24000
        chart_metrics.series[1].marker.symbol = "square"
        chart_metrics.series[1].marker.size = 5

    # Data Overshoot (kolom 9, derajat °) di Sumbu Y Sekunder
    chart_overshoot = LineChart()
    data_overshoot = Reference(ws_data, min_col=9, min_row=1, max_row=num_rows)
    chart_overshoot.add_data(data_overshoot, titles_from_data=True)
    chart_overshoot.set_categories(cats_time)

    # Konfigurasi Sumbu Y Sekunder (Menampilkan Angka Skala 0, 5, 10, 15, 20...)
    chart_overshoot.y_axis.title = "Simpangan Overshoot (derajat °)"
    chart_overshoot.y_axis.axId = 200
    chart_overshoot.y_axis.crosses = "max"  # Tampilkan sumbu Y sekunder di kanan
    chart_overshoot.y_axis.delete = False
    chart_overshoot.y_axis.tickLblPos = "nextTo"
    chart_overshoot.y_axis.majorTickMark = "out"
    chart_overshoot.y_axis.number_format = '0.0'
    chart_overshoot.y_axis.scaling.min = 0

    if chart_overshoot.series:
        chart_overshoot.series[0].graphicalProperties.line.solidFill = "385723" # Green (Overshoot)
        chart_overshoot.series[0].graphicalProperties.line.width = 24000
        chart_overshoot.series[0].marker.symbol = "triangle"
        chart_overshoot.series[0].marker.size = 5

    # Gabungkan chart Overshoot ke Sumbu Sekunder chart_metrics
    chart_metrics += chart_overshoot

    ws_summary.add_chart(chart_metrics, "B34")

    # 3. Grafik Posisi Kereta (cmX vs Waktu)
    chart_cmx = LineChart()
    chart_cmx.title = "Posisi Kereta Pendulum (cmX vs Waktu)"
    chart_cmx.style = 12
    chart_cmx.legend = None
    chart_cmx.y_axis.title = "Posisi Kereta cmX (cm)"
    chart_cmx.x_axis.title = "Waktu (detik)"
    chart_cmx.x_axis.tickLblSkip = 50
    chart_cmx.width = 20
    chart_cmx.height = 10

    chart_cmx.y_axis.delete = False
    chart_cmx.y_axis.tickLblPos = "nextTo"
    chart_cmx.y_axis.majorTickMark = "out"

    chart_cmx.x_axis.delete = False
    chart_cmx.x_axis.tickLblPos = "nextTo"
    chart_cmx.x_axis.majorTickMark = "out"

    data_cmx = Reference(ws_data, min_col=5, min_row=1, max_row=num_rows)
    chart_cmx.add_data(data_cmx, titles_from_data=True)
    chart_cmx.set_categories(cats_time)

    if chart_cmx.series:
        chart_cmx.series[0].graphicalProperties.line.solidFill = "ED7D31" # Orange
        chart_cmx.series[0].graphicalProperties.line.width = 20000

    ws_summary.add_chart(chart_cmx, "B56")

    # Simpan workbook
    wb.save(excel_path)
    print(f"[EXCEL] File Excel berhasil dibuat: {excel_path}")
    return excel_path


def batch_convert_all_logs(logs_dir: str = "logs"):
    """Konversi semua file .csv di folder logs atau folder saat ini menjadi .xlsx."""
    if not os.path.exists(logs_dir):
        # Jika folder logs tidak ditemukan tetapi ada file csv di folder saat ini
        logs_dir = "."

    csv_files = [f for f in os.listdir(logs_dir) if f.endswith('.csv')]
    if not csv_files and logs_dir == "logs" and os.path.exists("."):
        csv_files = [f for f in os.listdir(".") if f.endswith('.csv')]
        if csv_files:
            logs_dir = "."

    if not csv_files:
        print(f"Tidak ada file CSV ditemukan untuk dikonversi.")
        return

    print(f"Menemukan {len(csv_files)} file CSV di '{logs_dir}'...")
    for f in csv_files:
        path = os.path.join(logs_dir, f)
        try:
            process_csv_and_export_excel(path)
        except Exception as e:
            print(f"Gagal mengonversi {f}: {e}")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        target = sys.argv[1]
        if os.path.isdir(target):
            batch_convert_all_logs(target)
        else:
            process_csv_and_export_excel(target)
    else:
        # Default: jika di folder GUI Pendulum (ada logs/), gunakan logs/. Jika sedang di dalam logs/, gunakan .
        if os.path.exists("logs") and os.path.isdir("logs"):
            batch_convert_all_logs("logs")
        else:
            batch_convert_all_logs(".")
