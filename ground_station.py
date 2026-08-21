import socket
import struct
import time

UDP_IP = ""
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(1.0)

# Must match TelemetryPckt_t exactly
# start(B) roll(f) pitch(f) altitude(f) latitude(f) longitude(f)
# throttle(f) m1(f) m2(f) m3(f) m4(f) armed(B) gps_fix(B) satellites(B) checksum(B)
TELE_FORMAT = '<B 10f 4B'
TELE_SIZE   = struct.calcsize(TELE_FORMAT)
TELE_START  = 0xCD

def calc_checksum(data):
    chk = 0
    for b in data[1:-1]:
        chk ^= b
    return chk

print(f"Ground station listening on port {UDP_PORT}")
print(f"Expected packet size: {TELE_SIZE} bytes")
print("-" * 60)

while True:
    try:
        data, addr = sock.recvfrom(1024)

        if len(data) != TELE_SIZE:
            print(f"Wrong packet size: {len(data)} expected {TELE_SIZE}")
            continue

        if data[0] != TELE_START:
            print(f"Wrong start byte: 0x{data[0]:02X}")
            continue

        expected = calc_checksum(data)
        if data[-1] != expected:
            print(f"Checksum fail: got 0x{data[-1]:02X} expected 0x{expected:02X}")
            continue

        fields = struct.unpack(TELE_FORMAT, data)
        start = fields[0]
        roll, pitch, altitude, latitude, longitude, throttle, m1, m2, m3, m4 = fields[1:11]
        armed, gps_fix, satellites, checksum = fields[11:15]

        arm_str = "ARMED" if armed else "DISARMED"
        fix_str = "FIX" if gps_fix else "NO FIX"

        print(f"[{arm_str}] "
              f"R:{roll:+6.1f}° P:{pitch:+6.1f}° "
              f"Alt:{altitude:6.1f}m "
              f"T:{throttle:5.1f}% "
              f"M1:{m1:5.1f} M2:{m2:5.1f} M3:{m3:5.1f} M4:{m4:5.1f} "
              f"GPS:{fix_str} Sats:{satellites} "
              f"({latitude:.6f}, {longitude:.6f})")

    except socket.timeout:
        print("... waiting for telemetry ...")
    except Exception as e:
        print(f"Error: {e}")