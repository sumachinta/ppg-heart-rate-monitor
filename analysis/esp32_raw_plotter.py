"""
ESP32 MAX30102 — Raw Signal Plotter
=====================================
Reads raw IR and Red ADC values from ESP32 over USB Serial
and plots them as live scrolling lines.

No BPM or SpO2 calculation here — raw data only.
Use this to visually inspect signal quality before deriving anything.

What you should see with finger on sensor:
  - IR:  large value (50,000–200,000) with a clear rhythmic ripple
  - Red: smaller value (20,000–80,000) with same rhythmic ripple
  - The ripple IS your pulse — each peak = one heartbeat

What you should see without finger:
  - Both values near zero or very low (< 5,000)

Requirements:
    pip install pyserial matplotlib

Usage:
    python esp32_raw_plotter.py /dev/cu.usbserial-XXXX
    (port auto-detects if not provided)
"""

import sys
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time

# ═══════════════════════════════════════════════════════════
#  SETTINGS — things you can tune
# ═══════════════════════════════════════════════════════════

BAUD_RATE = 115200      # must match Serial.begin() in .ino

# How many samples to show in the scrolling window
# At 100Hz (PRINT_INTERVAL_MS=10 in .ino), samples arrive every 10ms
# 500 samples × 10ms = 5 seconds of history visible
WINDOW_SIZE = 250

# This is set as SAMPLE_RATE in the ino file
SAMPLE_RATE = 100 #100HZ 

# Serial port — set manually or leave None for auto-detect
PORT = "/dev/cu.usbserial-022B0C0C"             # e.g. "/dev/cu.usbserial-1420"

# Finger detection threshold — same as .ino
# IR above this = finger present
FINGER_THRESHOLD = 50000

# ═══════════════════════════════════════════════════════════

# ── Auto-detect ESP32 serial port ────────────────────────
def find_esp32_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description + p.hwid).lower()
        if any(x in desc for x in ['cp210', 'ch340', 'ftdi', 'uart', 'usb serial', 'usbserial']):
            return p.device
    if ports:
        return ports[0].device   # fallback: first available port
    return None

# ── Circular buffers — hold last WINDOW_SIZE samples ─────
# deque with maxlen automatically drops oldest when full
buf_ir  = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
buf_red = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)

# Latest parsed values (updated each serial read)
latest = {'ir': 0, 'red': 0}

# ── Open serial connection ────────────────────────────────
port = sys.argv[1] if len(sys.argv) > 1 else (PORT or find_esp32_port())

if port is None:
    print("ERROR: No serial port found.")
    print("Plug in ESP32 or pass port as argument: python esp32_raw_plotter.py /dev/cu.usbserial-XXXX")
    print("\nAvailable ports:")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device}  —  {p.description}")
    sys.exit(1)

print(f"Opening {port} at {BAUD_RATE} baud...")
try:
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    ser.reset_input_buffer()    # discard any stale bytes
    time.sleep(2)               # wait for ESP32 to reset after serial connect
    print("Connected. Waiting for data...")
except serial.SerialException as e:
    print(f"ERROR: Could not open port — {e}")
    sys.exit(1)

# ── Parse one incoming serial line ───────────────────────
def parse_line(line):
    """
    Expects lines in format:  DATA,<ir>,<red>
    Lines starting with # or HEADER are comments — ignored.
    Returns True if a valid DATA line was parsed.
    """
    line = line.strip()

    if not line.startswith("DATA,"):
        # Print comments/status lines to terminal for visibility
        if line and not line.startswith("HEADER"):
            print(line)
        return False

    try:
        # Split "DATA,187432,31205" → ['DATA', '187432', '31205']
        parts = line.split(",")
        latest['ir']  = int(parts[1])
        latest['red'] = int(parts[2])
        return True
    except (IndexError, ValueError):
        return False   # malformed line, skip

# ── Plot layout ───────────────────────────────────────────
fig, (ax_ir, ax_red) = plt.subplots(2, 1, figsize=(12, 7))
fig.patch.set_facecolor('#0d1318')
fig.suptitle('MAX30102 Raw Signal — IR & Red ADC Counts',
             color='white', fontsize=13, fontweight='bold')

def style_axis(ax, title):
    ax.set_facecolor('#080c10')
    ax.set_title(title, color='#8899aa', fontsize=9, loc='left', pad=6)
    ax.set_ylabel('ADC counts (0 – 262,144)', color='#556677', fontsize=8)
    ax.tick_params(colors='#445566', labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor('#1a2530')
    ax.grid(True, color='#1a2530', linewidth=0.5, linestyle='--')
    ax.set_xlim(0, WINDOW_SIZE)

style_axis(ax_ir,  'IR (infrared)  — primary signal for heart rate detection')
style_axis(ax_red, 'Red — used alongside IR to calculate SpO2')

# Draw the finger-detection threshold line on IR panel
ax_ir.axhline(y=FINGER_THRESHOLD, color='#ffd60a', lw=0.8,
              linestyle=':', alpha=0.7, label=f'Finger threshold ({FINGER_THRESHOLD:,})')
ax_ir.legend(loc='upper right', facecolor='#0d1318',
             edgecolor='#1a2530', labelcolor='white', fontsize=8)

x_axis = list(range(WINDOW_SIZE))

# Line objects — updated each frame
line_ir,  = ax_ir.plot( x_axis, list(buf_ir),  color='#00e5ff', lw=1.0)
line_red, = ax_red.plot(x_axis, list(buf_red), color='#ff4d6d', lw=1.0)

# Status bar at bottom of figure
status_text = fig.text(0.5, 0.01, 'Waiting...', ha='center',
                       color='#556677', fontsize=8, fontfamily='monospace')

plt.tight_layout(rect=[0, 0.04, 1, 0.96])

# ── Animation update — called every PLOT_INTERVAL_MS ─────
PLOT_INTERVAL_MS = 50   # how often matplotlib redraws (ms)
                        # independent of serial data rate
                        # 50ms = 20 redraws/sec, smooth enough

def update(frame):
    # Drain all pending serial bytes before redrawing
    # This prevents the buffer from backing up if plot is slow
    while ser.in_waiting > 0:
        try:
            raw = ser.readline().decode('utf-8', errors='replace')
            if parse_line(raw):
                buf_ir.append(latest['ir'])
                buf_red.append(latest['red'])
        except Exception:
            pass

    # Update plot data
    line_ir.set_ydata(list(buf_ir))
    line_red.set_ydata(list(buf_red))

    # Auto-scale Y axis to the actual data range (with 10% padding)
    for buf, ax in [(buf_ir, ax_ir), (buf_red, ax_red)]:
        vals = [v for v in buf if v > 0]
        if vals:
            mn, mx = min(vals), max(vals)
            pad = max((mx - mn) * 0.1, 500)   # at least 500 counts of padding
            ax.set_ylim(mn - pad, mx + pad)

    # Status bar: show current values and finger state
    finger_on = latest['ir'] > FINGER_THRESHOLD
    finger_str = "FINGER ON ✓" if finger_on else "NO FINGER"
    status_text.set_text(
        f"{finger_str}   |   IR = {latest['ir']:,}   Red = {latest['red']:,}"
    )
    status_text.set_color('#69ff47' if finger_on else '#ff4d6d')

    return line_ir, line_red, status_text

ani = animation.FuncAnimation(
    fig, update,
    interval=PLOT_INTERVAL_MS,
    blit=False,
    cache_frame_data=False
)

try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("Serial port closed.")
