"""
ESP32 MAX30102 — Data Recorder
================================
Records a fixed duration of IR + Red samples to a CSV file.
Use this to capture labelled datasets for algorithm testing.

Usage (structured — recommended):
    python record_sensor_data.py --subject suma --finger middle --condition normal
    python record_sensor_data.py --subject pavan --finger index --condition hyper --ref-hr 153

Usage (freeform label):
    python record_sensor_data.py --label my_custom_label --duration 30

Output (structured):
    ../data/12_middle_suma_normal_20260407_162501.csv
    ../data/13_index_pavan_hyper_appleHR153_20260407_163000.csv

Structured args auto-increment the sequence number from existing files in ../data/.

CSV format:
    sample_index, timestamp_ms, ir, red
"""

import sys
import re
import serial
import serial.tools.list_ports
import argparse
import csv
import os
import time
from datetime import datetime

# ═══════════════════════════════════════════════════════════
#  SETTINGS
# ═══════════════════════════════════════════════════════════

PORT       = "/dev/cu.usbserial-022B0C0C"
BAUD_RATE  = 115200
OUTPUT_DIR = "../data"   # folder where CSVs are saved

# ═══════════════════════════════════════════════════════════

def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if any(x in (p.description + p.hwid).lower()
               for x in ['cp210', 'ch340', 'ftdi', 'usbserial']):
            return p.device
    return ports[0].device if ports else None


def next_seq(data_dir: str) -> int:
    """Return the next available sequence number based on existing files."""
    os.makedirs(data_dir, exist_ok=True)
    nums = []
    for name in os.listdir(data_dir):
        m = re.match(r'^(\d+)_', name)
        if m:
            nums.append(int(m.group(1)))
    return max(nums) + 1 if nums else 0


def build_label(args) -> str:
    """Build a structured label from --subject/--finger/--condition/--ref-hr."""
    parts = [args.finger, args.subject, args.condition]
    if args.ref_hr:
        parts.append(f"appleHR{args.ref_hr}")
    return "_".join(parts)


def parse_line(line):
    line = line.strip()
    if not line.startswith("DATA,"):
        return None
    try:
        parts = line.split(",")
        return int(parts[1]), int(parts[2])   # ir, red
    except (IndexError, ValueError):
        return None


def record(label: str, duration: int, port: str, seq: int | None = None):
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if seq is not None:
        basename = f"{seq:02d}_{label}_{timestamp}.csv"
    else:
        basename = f"{label}_{timestamp}.csv"
    filename = os.path.join(OUTPUT_DIR, basename)

    print(f"\n  Label    : {label}")
    print(f"  Duration : {duration}s")
    print(f"  Port     : {port}")
    print(f"  Output   : {filename}")

    print(f"\nOpening serial port...")
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    ser.reset_input_buffer()
    time.sleep(2)

    print(f"Ready. Starting recording in 3 seconds — place finger now if needed...")
    for i in range(3, 0, -1):
        print(f"  {i}...")
        time.sleep(1)

    print(f"RECORDING for {duration}s  (Ctrl+C to stop early)\n")

    start_time  = time.time()
    start_ms    = int(start_time * 1000)
    sample_idx  = 0
    rows        = []

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break

            if ser.in_waiting > 0:
                try:
                    raw    = ser.readline().decode('utf-8', errors='replace')
                    result = parse_line(raw)
                    if result:
                        ir, red = result
                        ts_ms   = int(time.time() * 1000) - start_ms
                        rows.append([sample_idx, ts_ms, ir, red])
                        sample_idx += 1

                        # Progress bar
                        pct  = elapsed / duration
                        bar  = int(pct * 40)
                        print(f"\r  [{('#' * bar):<40}] {elapsed:.1f}/{duration}s  "
                              f"n={sample_idx}  IR={ir:,}  Red={red:,}  ", end='', flush=True)
                except Exception:
                    pass

    except KeyboardInterrupt:
        print("\n\nStopped early by user.")

    ser.close()

    # Save CSV
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['sample_index', 'timestamp_ms', 'ir', 'red'])
        writer.writerows(rows)

    duration_actual = rows[-1][1] / 1000 if rows else 0
    sample_rate_est = sample_idx / duration_actual if duration_actual > 0 else 0

    print(f"\n\n  Saved {sample_idx} samples → {filename}")
    print(f"  Actual duration : {duration_actual:.1f}s")
    print(f"  Estimated rate  : {sample_rate_est:.1f} Hz\n")


# ═══════════════════════════════════════════════════════════

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Record MAX30102 sensor data to CSV")

    # Structured naming (recommended)
    parser.add_argument('--subject',   type=str, help='Subject name (e.g. suma, pavan)')
    parser.add_argument('--finger',    type=str, choices=['index', 'middle', 'ring', 'thumb'],
                        help='Finger used on sensor')
    parser.add_argument('--condition', type=str, help='Recording condition (e.g. normal, hyper, rest)')
    parser.add_argument('--ref-hr',    type=int, dest='ref_hr', default=None,
                        help='Reference heart rate from Apple Watch (e.g. 76)')

    # Freeform fallback
    parser.add_argument('--label',    type=str, default=None,
                        help='Freeform label (overrides structured args)')

    parser.add_argument('--duration', type=int, default=30,
                        help='Recording duration in seconds (default: 30)')
    parser.add_argument('--port',     type=str, default=None,
                        help='Serial port (auto-detects if not set)')
    args = parser.parse_args()

    port = args.port or PORT or find_port()
    if not port:
        print("ERROR: No serial port found.")
        sys.exit(1)

    if args.label:
        record(args.label, args.duration, port, seq=None)
    elif args.subject and args.finger and args.condition:
        seq   = next_seq(OUTPUT_DIR)
        label = build_label(args)
        record(label, args.duration, port, seq=seq)
    else:
        parser.error("Provide either --label, or all of --subject, --finger, and --condition")
