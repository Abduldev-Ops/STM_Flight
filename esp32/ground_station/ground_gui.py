"""
Ground Station GUI for STM32 FreeRTOS Flight Controller
Receives TelemetryPckt_t over UDP, renders artificial horizon,
altitude history, motor outputs, and link/GPS/arm status.

Packet format (per project handoff doc):
  '<B10f4B' = start(1B)
              + roll, pitch, altitude, lat, lon, throttle, m1, m2, m3, m4 (10 floats)
              + armed, gps_fix, satellites, checksum (4B)

Run:
  python ground_station_gui.py --port 5005
  python ground_station_gui.py --port 5005 --no-checksum   (while confirming checksum scheme)
"""

import argparse
import socket
import struct
import threading
import time
from collections import deque


import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.transforms as mtransforms
import matplotlib.patches as mpatches

PACKET_START = 0xCD
PACKET_FORMAT = '<B 10f 4B'
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

STALE_TIMEOUT_S = 1.0     # no packet for this long -> treat link as down, show NO SIGNAL
HISTORY_SECONDS = 20      # rolling window for the altitude plot
ASSUMED_RATE_HZ = 20      # NOT confirmed for telemetry specifically - adjust if your
                          # STM32->ESP32 telemetry task runs at a different rate than
                          # the 20ms RC uplink cadence documented elsewhere in the project
HISTORY_LEN = HISTORY_SECONDS * ASSUMED_RATE_HZ


def calc_checksum(payload_bytes):
    """XOR of all bytes except start and checksum itself.

    This matches the scheme already used for the RC uplink checksum
    (calcCheckSum in task_rc.c / the ESP32 sketch). CONFIRM this matches
    your firmware's actual telemetry checksum function before trusting
    the validity filtering below - if it doesn't match, every packet
    will be silently dropped as "corrupted" even though the data is fine.
    """
    chk = 0
    for b in payload_bytes[1:-1]:
        chk ^= b
    return chk


class TelemetryState:
    """Thread-safe holder for the latest decoded packet plus short history.

    The UDP listener thread only ever calls update(). The GUI thread only
    ever calls snapshot(). Neither touches the other's data directly -
    that's what the lock is for.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.roll = 0.0
        self.pitch = 0.0
        self.altitude = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.throttle = 0.0
        self.motors = [0.0, 0.0, 0.0, 0.0]
        self.armed = 0
        self.gps_fix = 0
        self.satellites = 0
        self.last_packet_time = 0.0
        self.alt_history = deque(maxlen=HISTORY_LEN)
        self.time_history = deque(maxlen=HISTORY_LEN)
        self.t0 = time.time()

    def update(self, unpacked):
        (start, roll, pitch, altitude, lat, lon, throttle,
         m1, m2, m3, m4, armed, gps_fix, satellites, checksum) = unpacked

        with self._lock:
            self.roll = roll
            self.pitch = pitch
            self.altitude = altitude
            self.lat = lat
            self.lon = lon
            self.throttle = throttle
            self.motors = [m1, m2, m3, m4]
            self.armed = armed
            self.gps_fix = gps_fix
            self.satellites = satellites
            self.last_packet_time = time.time()
            self.alt_history.append(altitude)
            self.time_history.append(self.last_packet_time - self.t0)

    def snapshot(self):
        with self._lock:
            stale = (
                (time.time() - self.last_packet_time) > STALE_TIMEOUT_S
                if self.last_packet_time else True
            )
            return dict(
                roll=self.roll, pitch=self.pitch, altitude=self.altitude,
                lat=self.lat, lon=self.lon, throttle=self.throttle,
                motors=list(self.motors), armed=self.armed,
                gps_fix=self.gps_fix, satellites=self.satellites,
                stale=stale,
                alt_history=list(self.alt_history),
                time_history=list(self.time_history),
            )


def udp_listener(sock, state, verify_checksum=True):
    """Runs in its own thread. Never touches matplotlib - only writes
    into `state`, which the GUI thread reads via snapshot()."""
    while True:
        try:
            data, _addr = sock.recvfrom(1024)
        except OSError:
            return  # socket closed (program exiting) - exit thread cleanly

        if len(data) != PACKET_SIZE:
            print(f"[DROP] size mismatch: got {len(data)} bytes, expected {PACKET_SIZE}")
            continue  # truncated/malformed - drop, next packet arrives shortly

        unpacked = struct.unpack(PACKET_FORMAT, data)
        start = unpacked[0]
        checksum = unpacked[-1]

        if start != PACKET_START:
            print(f"[DROP] bad start byte: 0x{start:02X}, expected 0x{PACKET_START:02X}")
            continue

        if verify_checksum:
            expected = calc_checksum(data)
            if checksum != expected:
                print(f"[DROP] checksum mismatch: got 0x{checksum:02X}, expected 0x{expected:02X}")
                continue  # corrupted packet - drop rather than display bad data

        state.update(unpacked)


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def draw_horizon(ax, roll_deg, pitch_deg):
    ax.clear()
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("Attitude", fontsize=10)

    # Pitch maps to vertical offset of the horizon line; clamped so extreme
    # pitch doesn't push the whole horizon off-screen and look "stuck".
    pitch_offset = max(-0.9, min(0.9, pitch_deg / 45.0))

    # Ground/sky as two large rectangles, rotated by roll and shifted by
    # pitch, then clipped to the axes view - the standard artificial-
    # horizon rendering trick.
    t = (mtransforms.Affine2D()
         .translate(0, pitch_offset)
         .rotate_deg(-roll_deg) + ax.transData)

    sky = mpatches.Rectangle((-2, 0), 4, 4, facecolor='#4a90d9', transform=t, zorder=0)
    ground = mpatches.Rectangle((-2, -4), 4, 4, facecolor='#8b5a2b', transform=t, zorder=0)
    ax.add_patch(sky)
    ax.add_patch(ground)

    horizon_line = plt.Line2D([-2, 2], [0, 0], color='white', linewidth=1.5,
                               transform=t, zorder=1)
    ax.add_line(horizon_line)

    # Fixed aircraft reference symbol - NOT transformed, always screen-centered
    ax.plot([-0.3, -0.08], [0, 0], color='yellow', linewidth=3, zorder=2)
    ax.plot([0.08, 0.3], [0, 0], color='yellow', linewidth=3, zorder=2)
    ax.plot([0], [0], marker='o', color='yellow', markersize=5, zorder=2)

    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)


def draw_motor_bars(ax, motors, armed):
    ax.clear()
    color = '#2ecc71' if armed else '#95a5a6'
    bars = ax.bar(['M1', 'M2', 'M3', 'M4'], motors, color=color)
    ax.set_ylim(0, 100)
    ax.set_title("Motor Output (%)", fontsize=10)
    for bar, val in zip(bars, motors):
        ax.text(bar.get_x() + bar.get_width() / 2, val + 2, f"{val:.0f}",
                 ha='center', fontsize=8)


def draw_status(ax, snap):
    ax.clear()
    ax.axis('off')

    if snap['stale']:
        ax.text(0.5, 0.5, "NO SIGNAL", ha='center', va='center',
                 fontsize=20, color='red', fontweight='bold',
                 transform=ax.transAxes)
        return

    armed_color = '#2ecc71' if snap['armed'] else '#e74c3c'
    armed_text = "ARMED" if snap['armed'] else "DISARMED"
    gps_text = (f"GPS: {snap['satellites']} sats (FIX)"
                if snap['gps_fix'] else "GPS: NO FIX")
    gps_color = '#2ecc71' if snap['gps_fix'] else '#e74c3c'

    lines = [
        (armed_text, armed_color, 16),
        (gps_text, gps_color, 12),
        (f"Lat: {snap['lat']:.6f}", 'black', 10),
        (f"Lon: {snap['lon']:.6f}", 'black', 10),
        (f"Alt: {snap['altitude']:.1f} m", 'black', 10),
        (f"Throttle: {snap['throttle']:.0f}%", 'black', 10),
    ]

    y = 0.9
    for text, color, size in lines:
        ax.text(0.05, y, text, fontsize=size, color=color,
                 fontweight='bold' if size >= 14 else 'normal',
                 transform=ax.transAxes)
        y -= 0.15


def main():
    parser = argparse.ArgumentParser(description="Flight controller ground station GUI")
    parser.add_argument('--port', type=int, default=5005, help="UDP port to listen on")
    parser.add_argument('--no-checksum', action='store_true',
                         help="Skip checksum validation - use while confirming your "
                              "firmware's telemetry checksum matches this script's")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', args.port))

    state = TelemetryState()
    listener_thread = threading.Thread(
        target=udp_listener, args=(sock, state, not args.no_checksum), daemon=True
    )
    listener_thread.start()

    fig = plt.figure(figsize=(10, 7))
    fig.suptitle("Drone Ground Station", fontsize=14)
    gs = fig.add_gridspec(2, 2)
    ax_horizon = fig.add_subplot(gs[0, 0])
    ax_altitude = fig.add_subplot(gs[0, 1])
    ax_motors = fig.add_subplot(gs[1, 0])
    ax_status = fig.add_subplot(gs[1, 1])

    def update(_frame):
        snap = state.snapshot()

        draw_horizon(ax_horizon, snap['roll'], snap['pitch'])

        ax_altitude.clear()
        ax_altitude.set_title("Altitude (m)", fontsize=10)
        if snap['time_history']:
            ax_altitude.plot(snap['time_history'], snap['alt_history'], color='#2980b9')
        ax_altitude.set_xlabel("t (s)", fontsize=8)

        draw_motor_bars(ax_motors, snap['motors'], snap['armed'])
        draw_status(ax_status, snap)

        return []

    ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)

    try:
        plt.tight_layout()
        plt.show()
    finally:
        sock.close()


if __name__ == '__main__':
    main()