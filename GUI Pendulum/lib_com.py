import serial
import serial.tools.list_ports
import struct
import time


def find_available_ports():
    """Mengembalikan daftar tuple (port, description) dari port serial yang tersedia."""
    try:
        ports = list(serial.tools.list_ports.comports())
        return [(p.device, p.description) for p in ports]
    except Exception:
        return []


def auto_detect_serial_port(preferred_port: str = "COM14"):
    """
    Cari port serial yang valid.
    1. Coba preferred_port jika tersedia.
    2. Jika preferred_port tidak ada, cari port USB / STM32 / UART.
    3. Jika ada port lain, kembalikan port pertama.
    4. Kembalikan None jika tidak ada port.
    """
    available = find_available_ports()
    if not available:
        return None

    devices = [p[0] for p in available]
    if preferred_port in devices:
        return preferred_port

    # Cari device yang tampak seperti STM32 / USB Serial
    for dev, desc in available:
        desc_upper = desc.upper()
        if any(k in desc_upper for k in ("STM", "USB", "CH340", "CP210", "FTDI", "VIRTUAL")):
            print(f"[Serial] Auto-detected hardware port: {dev} ({desc})")
            return dev

    # Fallback to first available port if hardware filter didn't match
    print(f"[Serial] Fallback to available port: {devices[0]} ({available[0][1]})")
    return devices[0]


def open_serial(port: str, baud: int, timeout: float = 0.0):
    """Buka port serial."""
    return serial.Serial(port, baud, timeout=timeout)


def make_packet(seq, ax, ay, rx, ry, buttons):
    """Buat paket joystick → STM32."""
    header = b'\xAA\x55'
    typ = 0x01
    body = struct.pack('<BBhhhhH', typ, seq, ax, ay, rx, ry, buttons)
    s = sum(body) % 65535
    chksum = struct.pack('<H', s)
    return header + body + chksum


def make_gains_packet(seq, K_TH, K_TH_D, K_X, K_X_D, K_X_INT):
    """
    Buat paket set gains → STM32.
    
    Format:
    - Header: 0xAA 0x55
    - Type: 0x02
    - Seq: 1 byte
    - Payload: 5 floats (20 bytes)
    - CRC: 2 bytes
    """
    header = b'\xAA\x55'
    typ = 0x02
    
    # Pack: type(1) + seq(1) + 5 floats(20)
    body = struct.pack('<BBfffff', 
                       typ, seq,
                       K_TH, K_TH_D, K_X, K_X_D, K_X_INT)
    
    # CRC = sum(body) % 65536
    s = sum(body) % 65536
    chksum = struct.pack('<H', s)
    
    return header + body + chksum


def send_gains(ser, K_TH, K_TH_D, K_X, K_X_D, K_X_INT, seq=0):
    """Kirim gains ke STM32 via serial."""
    packet = make_gains_packet(seq, K_TH, K_TH_D, K_X, K_X_D, K_X_INT)
    ser.write(packet)
    print(f"[TX] Gains sent: K_TH={K_TH:.2f}, K_TH_D={K_TH_D:.4f}, "
          f"K_X={K_X:.2f}, K_X_D={K_X:.2f}, K_X_INT={K_X_INT:.2f}")


def make_reset_packet(seq=0):
    """
    Buat paket reset → STM32.
    
    Format:
    - Header: 0xAA 0x55
    - Type: 0x03
    - Seq: 1 byte
    - Payload: NONE (0 bytes)
    - CRC: 2 bytes
    """
    header = b'\xAA\x55'
    typ = 0x03
    
    # Pack: type(1) + seq(1), no payload
    body = struct.pack('<BB', typ, seq)
    
    # CRC = sum(body) % 65536
    s = sum(body) % 65536
    chksum = struct.pack('<H', s)
    
    return header + body + chksum


def send_reset(ser, seq=0):
    """Kirim reset command ke STM32 via serial."""
    packet = make_reset_packet(seq)
    
    # Debug: print packet bytes
    print(f"[TX] Reset packet ({len(packet)} bytes): {packet.hex()}")
    
    ser.write(packet)
    print("[TX] Reset command sent to STM32")


def make_physics_packet(seq, length_m, mass_kg):
    """
    Buat paket parameter fisik pendulum → STM32.

    Format:
    - Header: 0xAA 0x55
    - Type: 0x04
    - Seq: 1 byte
    - Payload: 2 floats (8 bytes): length_m, mass_kg
    - CRC: 2 bytes
    Total: 14 bytes
    """
    header = b'\xAA\x55'
    typ = 0x04

    # Pack: type(1) + seq(1) + 2 floats(8)
    body = struct.pack('<BBff',
                       typ, seq,
                       float(length_m), float(mass_kg))

    s = sum(body) % 65536
    chksum = struct.pack('<H', s)

    return header + body + chksum


def send_physics(ser, length_m, mass_kg, seq=0):
    """Kirim parameter fisik pendulum ke STM32 via serial (paket tipe 0x04)."""
    packet = make_physics_packet(seq, length_m, mass_kg)
    ser.write(packet)
    print(f"[TX] Physics params: length={length_m:.4f} m, mass={mass_kg:.4f} kg  "
          f"({len(packet)} bytes: {packet.hex()})")




def make_swing_packet(seq, kE, kx_swing, margin_E, catch_deg, catch_vel):
    """
    Buat paket parameter swing-up & handover -> STM32 (tipe 0x05).

    Format:
    - Header: 0xAA 0x55
    - Type: 0x05
    - Seq: 1 byte
    - Payload: 5 floats (20 bytes): kE, kx_swing, margin_E, catch_deg, catch_vel
    - CRC: 2 bytes
    Total: 26 bytes
    """
    header = b'\xAA\x55'
    typ = 0x05

    body = struct.pack('<BBfffff',
                       typ, seq,
                       float(kE), float(kx_swing), float(margin_E),
                       float(catch_deg), float(catch_vel))

    s = sum(body) % 65536
    chksum = struct.pack('<H', s)

    return header + body + chksum


def send_swing_params(ser, kE, kx_swing, margin_E, catch_deg, catch_vel, seq=0):
    """Kirim parameter swing-up & handover ke STM32 via serial (paket tipe 0x05)."""
    packet = make_swing_packet(seq, kE, kx_swing, margin_E, catch_deg, catch_vel)
    ser.write(packet)
    print(f"[TX] Swing params: kE={kE:.2f}, kx={kx_swing:.2f}, margin={margin_E:.2f}, "
          f"catch_deg={catch_deg:.1f} deg, catch_vel={catch_vel:.2f} rad/s "
          f"({len(packet)} bytes: {packet.hex()})")


def read_control_status(ser, callback=None, ack_callback=None, reset_ack_callback=None, swing_ack_callback=None, debug: bool = False):
    """
    Thread pembaca data dari STM32.
    
    Mendukung 3 jenis packet:
    1. Control Status (0xAA 0xCC) - existing
    2. Gains ACK (0xAA 0xDD) - gains confirmation
    3. Reset ACK (0xAA 0xEE) - reset confirmation (NEW!)
    
    Format control_status: <Idddddd>
    uint32 logtick + 6x double (degree, cmX, setspeed, reserved[3])
    
    Format gains_ack: 5x float (K_TH, K_TH_D, K_X, K_X_D, K_X_INT)
    
    Format reset_ack: 1 byte status
    """
    HEADER_STATUS = b'\xAA\xCC'
    HEADER_ACK = b'\xAA\xDD'
    HEADER_RESET = b'\xAA\xEE'
    HEADER_SWING = b'\xAA\xBB'
    
    status_total_len = 4 + 60 + 2  # header + payload + crc = 58
    ack_total_len = 2 + 20 + 2     # header + payload + crc = 24
    reset_total_len = 2 + 1 + 2    # header + payload + crc = 5
    swing_total_len = 2 + 20 + 2   # header + payload + crc = 24
    
    fmt_status = "<Iddddddd"   # uint32 + 7x double
    fmt_ack = "<fffff"        # 5x float

    buffer = bytearray()

    while True:
        try:
            if ser is None or not getattr(ser, "is_open", False):
                time.sleep(0.1)
                continue
            chunk = ser.read(128)
        except Exception as e:
            print(f"[Serial RX Warning] Port terputus / error: {e}")
            break

        if not chunk:
            time.sleep(0.01)
            continue
        buffer.extend(chunk)

        # Process buffer
        while len(buffer) >= 4:  # minimal header length
            # Try all headers
            idx_status = buffer.find(HEADER_STATUS)
            idx_ack = buffer.find(HEADER_ACK)
            idx_reset = buffer.find(HEADER_RESET)
            idx_swing = buffer.find(HEADER_SWING)
            
            # Pilih header yang muncul duluan
            first_idx = min([i for i in [idx_status, idx_ack, idx_reset, idx_swing] if i >= 0], default=-1)
            
            if first_idx < 0:
                buffer.clear()
                break
            
            # Process Control Status
            if first_idx == idx_status:
                if idx_status > 0:
                    del buffer[:idx_status]
                
                if len(buffer) < status_total_len:
                    break
                
                pkt = buffer[:status_total_len]
                del buffer[:status_total_len]
                
                # Validate CRC
                crc_recv = pkt[-2] | (pkt[-1] << 8)
                crc_calc = sum(pkt[4:-2]) & 0xFFFF
                if crc_recv != crc_calc:
                    if debug:
                        print("CRC mismatch (control_status)")
                    continue
                
                # Parse data
                data = pkt[4:-2]
                unpacked = struct.unpack(fmt_status, data)
                logtick = unpacked[0]
                degree, cmX, setspeed = unpacked[1:4]
                r1, r2, r3, r4 = unpacked[4:8] #?
                
                if debug:
                    print(f"[RX] tick={logtick:8d} deg={degree:8.3f} "
                          f"cmX={cmX:8.3f} set={setspeed:8.3f}")
                
                if callback is not None:
                    try:
                        callback((logtick, degree, cmX, setspeed, r1, r2, r3, r4))
                    except Exception as e:
                        print(f"[RX Callback Error] {e}")
            
            # Process Gains ACK
            elif first_idx == idx_ack:
                if idx_ack > 0:
                    del buffer[:idx_ack]
                
                if len(buffer) < ack_total_len:
                    break
                
                pkt = buffer[:ack_total_len]
                del buffer[:ack_total_len]
                
                # Validate CRC
                crc_recv = pkt[-2] | (pkt[-1] << 8)
                crc_calc = sum(pkt[2:-2]) & 0xFFFF
                if crc_recv != crc_calc:
                    if debug:
                        print("CRC mismatch (gains_ack)")
                    continue
                
                # Parse gains
                data = pkt[2:-2]
                gains = struct.unpack(fmt_ack, data)
                K_TH, K_TH_D, K_X, K_X_D, K_X_INT = gains
                
                if debug:
                    print(f"[RX] Gains ACK: K_TH={K_TH:.2f}, K_TH_D={K_TH_D:.4f}, "
                          f"K_X={K_X:.2f}, K_X_D={K_X:.2f}, K_X_INT={K_X_INT:.2f}")
                
                if ack_callback is not None:
                    try:
                        ack_callback(gains)
                    except Exception as e:
                        print(f"[ACK Callback Error] {e}")
            
            # Process Reset ACK (NEW!)
            elif first_idx == idx_reset:
                if idx_reset > 0:
                    del buffer[:idx_reset]
                
                if len(buffer) < reset_total_len:
                    break
                
                pkt = buffer[:reset_total_len]
                del buffer[:reset_total_len]
                
                # Validate CRC
                crc_recv = pkt[-2] | (pkt[-1] << 8)
                crc_calc = sum(pkt[2:-2]) & 0xFFFF
                if crc_recv != crc_calc:
                    if debug:
                        print("CRC mismatch (reset_ack)")
                    continue
                
                # Parse status
                status = pkt[2]
                
                print(f"[RX] *** RESET ACK RECEIVED *** status={status}")
                
                if reset_ack_callback is not None:
                    try:
                        reset_ack_callback(status)
                    except Exception as e:
                        print(f"[Reset ACK Callback Error] {e}")
            
            # Process Swing ACK (NEW!)
            elif first_idx == idx_swing:
                if idx_swing > 0:
                    del buffer[:idx_swing]
                
                if len(buffer) < swing_total_len:
                    break
                
                pkt = buffer[:swing_total_len]
                del buffer[:swing_total_len]
                
                # Validate CRC
                crc_recv = pkt[-2] | (pkt[-1] << 8)
                crc_calc = sum(pkt[2:-2]) & 0xFFFF
                if crc_recv != crc_calc:
                    if debug:
                        print("CRC mismatch (swing_ack)")
                    continue
                
                # Parse swing params: 5 floats
                data = pkt[2:-2]
                swing_params = struct.unpack("<fffff", data)
                kE, kx_swing, margin_E, catch_deg, catch_vel = swing_params
                
                if debug:
                    print(f"[RX] Swing ACK: kE={kE:.2f}, kx={kx_swing:.2f}, margin={margin_E:.2f}, "
                          f"deg={catch_deg:.1f}, vel={catch_vel:.2f}")
                
                if swing_ack_callback is not None:
                    try:
                        swing_ack_callback(swing_params)
                    except Exception as e:
                        print(f"[Swing ACK Callback Error] {e}")
            
            else:
                # No valid header found, clear garbage
                buffer.clear()
                break