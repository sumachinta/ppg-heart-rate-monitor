"""
ESP32 MAX30102 — Live Serial Plotter
=====================================
Reads CSV debug output from ESP32 over USB Serial and plots:
  - IR Raw signal
  - Red Raw signal
  - BPM (instantaneous) and BeatAvg (rolling 4-beat)
  - SpO2 value

Requirements:
    pip install pyserial matplotlib

Usage:
    1. Connect ESP32 via USB
    2. Find your port:  ls /dev/tty.usbserial* /dev/cu.usbserial*
    3. Set PORT below (or pass as argument: python plotter.py /dev/cu.usbserial-XXXX)
    4. Run: python plotter.py
"""

import sys
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time

# ── Config ────────────────────────────────────────────────
BAUD_RATE   = 115200
WINDOW_SIZE = 120       # number of samples shown (120 × 500ms = 60 seconds)
PORT        = None      # auto-detect if None, or set e.g. "/dev/cu.usbserial-1420"

# ── Auto-detect port ──────────────────────────────────────
def find_esp32_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description + p.hwid).lower()
        if any(x in desc for x in ['cp210', 'ch340', 'ftdi', 'uart', 'usb serial', 'usbserial']):
            return p.device
    # fallback: return first available port
    if ports:
        return ports[0].device
    return None

# ── Data buffers ──────────────────────────────────────────
buf_ir      = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
buf_red     = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
buf_bpm     = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
buf_avg     = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
buf_spo2    = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)

latest = {
    'ir': 0, 'red': 0, 'bpm': 0.0,
    'avg': 0, 'spo2': 0, 'spo2Valid': 0,
    'hrValid': 0, 'fingerOn': 0
}

# ── Serial setup ──────────────────────────────────────────
port = sys.argv[1] if len(sys.argv) > 1 else (PORT or find_esp32_port())

if port is None:
    print("ERROR: No serial port found. Connect ESP32 or specify port as argument.")
    print("Available ports:")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device}  —  {p.description}")
    sys.exit(1)

print(f"Connecting to {port} at {BAUD_RATE} baud...")
try:
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    time.sleep(2)   # wait for ESP32 reset
    ser.reset_input_buffer()
    print("Connected. Waiting for data...")
except serial.SerialException as e:
    print(f"ERROR opening port: {e}")
    sys.exit(1)

# ── Parse incoming line ───────────────────────────────────
def parse_line(line):
    line = line.strip()
    if not line.startswith("DATA,"):
        return False
    try:
        parts = line.split(",")
        # DATA,irRaw,redRaw,bpm,beatAvg,spo2,spo2Valid,hrValid,fingerOn
        latest['ir']       = int(parts[1])
        latest['red']      = int(parts[2])
        latest['bpm']      = float(parts[3])
        latest['avg']      = int(parts[4])
        latest['spo2']     = int(parts[5])
        latest['spo2Valid']= int(parts[6])
        latest['hrValid']  = int(parts[7])
        latest['fingerOn'] = int(parts[8])
        return True
    except (IndexError, ValueError):
        return False

# ── Plot setup ────────────────────────────────────────────
fig, axes = plt.subplots(4, 1, figsize=(12, 11))
fig.patch.set_facecolor('#0d1318')
fig.suptitle('ESP32 MAX30102 — Live Debug', color='white',
             fontsize=13, fontweight='bold', y=0.98)

COLORS = {
    'ir':   '#00e5ff',
    'red':  '#ff4d6d',
    'bpm':  '#ffd60a',
    'avg':  '#69ff47',
    'spo2': '#00e5ff',
}

def style_ax(ax, title, ylabel):
    ax.set_facecolor('#080c10')
    ax.set_title(title, color='#8899aa', fontsize=9, loc='left', pad=6)
    ax.set_ylabel(ylabel, color='#556677', fontsize=8)
    ax.tick_params(colors='#445566', labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor('#1a2530')
    ax.grid(True, color='#1a2530', linewidth=0.5, linestyle='--')
    ax.set_xlim(0, WINDOW_SIZE)

# Panel 0: IR signal
ax0 = axes[0]
style_ax(ax0, 'IR SIGNAL (infrared)', 'ADC counts')
line_ir, = ax0.plot(buf_ir, color=COLORS['ir'], lw=1.2, label='IR')

# Panel 1: Red signal
ax1 = axes[1]
style_ax(ax1, 'RED SIGNAL', 'ADC counts')
line_red, = ax1.plot(buf_red, color=COLORS['red'], lw=1.2, label='Red')

# Panel 2: BPM instantaneous + rolling avg
ax2 = axes[2]
style_ax(ax2, 'HEART RATE  (yellow = instant BPM, green = 4-beat avg)', 'BPM')
line_bpm, = ax2.plot(buf_bpm, color=COLORS['bpm'], lw=1.2, label='Instant BPM')
line_avg, = ax2.plot(buf_avg, color=COLORS['avg'], lw=2.0, label='Beat Avg', alpha=0.9)
ax2.axhline(y=60,  color='#334455', lw=0.8, linestyle=':')
ax2.axhline(y=100, color='#334455', lw=0.8, linestyle=':')
ax2.legend(loc='upper right', facecolor='#0d1318', edgecolor='#1a2530',
           labelcolor='white', fontsize=8)

# Panel 3: SpO2
ax3 = axes[3]
style_ax(ax3, 'SpO₂  (cyan = value, only meaningful when spo2Valid=1)', '% Saturation')
line_spo2, = ax3.plot(buf_spo2, color=COLORS['spo2'], lw=1.5)
ax3.axhline(y=95,  color='#69ff47', lw=0.8, linestyle=':', alpha=0.6)
ax3.axhline(y=90,  color='#ffd60a', lw=0.8, linestyle=':', alpha=0.6)

# Status text overlay
status_text = fig.text(0.5, 0.01, 'Waiting for data...', ha='center',
                       color='#556677', fontsize=8, fontfamily='monospace')

plt.tight_layout(rect=[0, 0.03, 1, 0.97])

# ── Animation update ──────────────────────────────────────
x_axis = list(range(WINDOW_SIZE))

def update(frame):
    # Read all available lines from serial
    lines_read = 0
    while ser.in_waiting > 0 and lines_read < 10:
        try:
            raw = ser.readline().decode('utf-8', errors='replace')
            if parse_line(raw):
                buf_ir.append(latest['ir'])
                buf_red.append(latest['red'])
                buf_bpm.append(latest['bpm'] if latest['fingerOn'] else 0)
                buf_avg.append(latest['avg'] if latest['fingerOn'] else 0)
                buf_spo2.append(latest['spo2'] if latest['spo2Valid'] else 0)
            else:
                # Print non-data lines (comments, status) to console
                raw = raw.strip()
                if raw:
                    print(raw)
        except Exception:
            pass
        lines_read += 1

    # Update plots
    line_ir.set_data(x_axis, list(buf_ir))
    line_red.set_data(x_axis, list(buf_red))
    line_bpm.set_data(x_axis, list(buf_bpm))
    line_avg.set_data(x_axis, list(buf_avg))
    line_spo2.set_data(x_axis, list(buf_spo2))

    # Auto-scale IR panel
    ir_vals = [v for v in buf_ir if v > 0]
    if ir_vals:
        mn, mx = min(ir_vals), max(ir_vals)
        pad = max((mx - mn) * 0.1, 500)
        ax0.set_ylim(mn - pad, mx + pad)

    # Auto-scale Red panel
    red_vals = [v for v in buf_red if v > 0]
    if red_vals:
        mn, mx = min(red_vals), max(red_vals)
        pad = max((mx - mn) * 0.1, 500)
        ax1.set_ylim(mn - pad, mx + pad)

    # Auto-scale BPM panel
    bpm_vals = [v for v in list(buf_bpm) + list(buf_avg) if v > 0]
    if bpm_vals:
        mn, mx = min(bpm_vals), max(bpm_vals)
        pad = max((mx - mn) * 0.15, 10)
        ax2.set_ylim(max(0, mn - pad), mx + pad)

    # Auto-scale SpO2 panel
    spo2_vals = [v for v in buf_spo2 if v > 0]
    if spo2_vals:
        mn, mx = min(spo2_vals), max(spo2_vals)
        pad = max((mx - mn) * 0.15, 2)
        ax3.set_ylim(max(0, mn - pad), min(105, mx + pad))

    # Status bar
    finger = "FINGER ON" if latest['fingerOn'] else "NO FINGER"
    hr_state = "HR VALID" if latest['hrValid'] else "stabilizing"
    spo2_state = f"SpO2 VALID ({latest['spo2']}%)" if latest['spo2Valid'] else "SpO2 invalid"
    status_text.set_text(
        f"{finger}   |   IR={latest['ir']}   Red={latest['red']}   "
        f"BPM={latest['bpm']:.0f}   Avg={latest['avg']}   "
        f"{hr_state}   {spo2_state}"
    )
    status_text.set_color('#69ff47' if latest['fingerOn'] else '#ff4d6d')

    return line_ir, line_red, line_bpm, line_avg, line_spo2, status_text


ani = animation.FuncAnimation(fig, update, interval=200, blit=False, cache_frame_data=False)

try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("Serial port closed.")
