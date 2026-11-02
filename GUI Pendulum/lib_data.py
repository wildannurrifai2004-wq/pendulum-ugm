import csv
import os
import time
import threading
import queue
from datetime import datetime


class DataLogger:
    """
    Logger data dari STM32 ke CSV.

    - Data masuk lewat handle_sample(tuple)
    - Penulisan ke file dipisah di thread worker + queue,
      supaya aman di-rate ~20 ms (50 Hz).
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

        self._worker_thread = threading.Thread(
            target=self._worker_loop,
            daemon=True
        )
        self._worker_thread.start()

    # ---------------- API ----------------

    def is_recording(self) -> bool:
        with self._lock:
            return self._recording

    def set_recording(self, enabled: bool):
        """
        Aktif/nonaktifkan rekam.
        Tiap kali ON akan buka file baru dengan timestamp.
        """
        with self._lock:
            if enabled == self._recording:
                return

            if enabled:
                self._open_new_file()
            else:
                self._close_file()

            self._recording = enabled

    def toggle_recording(self):
        with self._lock:
            new_state = not self._recording
        self.set_recording(new_state)

    def handle_sample(self, sample_tuple):
        """
        Dipanggil dari thread pembaca serial (lib_com.read_control_status).
        sample_tuple: (logtick, degree, cmX, setspeed, r1, r2, r3)
        """
        now = time.time()
        # push ke queue bersama timestamp PC saat data diterima
        self._queue.put((now, sample_tuple))

    # ---------------- internal ----------------

    def _open_new_file(self):
        if self._file is not None:
            self._close_file()

        now = time.time()
        self._start_time = now
        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
        filename = os.path.join(self.base_dir, f"log_{ts}.csv")
        self._file = open(filename, "w", newline="")
        self._writer = csv.writer(self._file)
        self._row_count = 0

        # header
        self._writer.writerow(
            ["datetime", "timestamp", "elapsed_s", "logtick", "degree", "cmX", "motor_rpm", "reserved1", "reserved2", "reserved3", "reserved4"]
        )
        print("Recording to:", filename)

    def _close_file(self):
        filename_to_export = None
        if self._file is not None:
            try:
                filename_to_export = self._file.name
                self._file.flush()
                self._file.close()
            except Exception:
                pass
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
                    print(f"[EXCEL] Auto Export error: {e}")
            threading.Thread(target=_export, daemon=True).start()

    def _worker_loop(self):
        """
        Loop penulis CSV. Selalu mengosongkan queue supaya
        tidak numpuk walau recording OFF.
        """
        while True:
            try:
                item = self._queue.get()   # blocking
                with self._lock:
                    if self._recording and self._writer is not None:
                        if isinstance(item, tuple) and len(item) == 2 and isinstance(item[0], float):
                            recv_time, sample = item
                        else:
                            recv_time = time.time()
                            sample = item

                        if self._start_time is None:
                            self._start_time = recv_time

                        elapsed_s = round(recv_time - self._start_time, 3)
                        dt_str = datetime.fromtimestamp(recv_time).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        ts_val = round(recv_time, 3)

                        # Konversi setspeed (index 3) ke RPM: 25000 clamp = 2400 RPM
                        sample_list = list(sample)
                        if len(sample_list) > 3:
                            try:
                                sample_list[3] = round(float(sample_list[3]) * (2400.0 / 25000.0), 2)
                            except (ValueError, TypeError):
                                pass

                        row = [dt_str, ts_val, elapsed_s] + sample_list
                        self._writer.writerow(row)
                        self._row_count += 1
                        # flush per 50 baris (~1 detik @ 50 Hz)
                        if (self._row_count % 50) == 0:
                            self._file.flush()
            except Exception as e:
                print(f"[DataLogger Error] {e}")
