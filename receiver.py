#!/usr/bin/env python3
"""Receive and display CSV telemetry sent by the ESP32."""

import socket


UDP_PORT = 5000
FIELDS = (
    "sequence,timestamp_us,accel_x_ms2,accel_y_ms2,accel_z_ms2,"
    "gyro_x_dps,gyro_y_dps,gyro_z_dps,pitch_deg,roll_deg,yaw_deg"
)


def main() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("0.0.0.0", UDP_PORT))
        print(f"Listening for ESP32 telemetry on UDP port {UDP_PORT}")
        print(FIELDS)

        while True:
            packet, sender = sock.recvfrom(2048)
            print(f"{sender[0]}: {packet.decode('utf-8').strip()}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nReceiver stopped")
