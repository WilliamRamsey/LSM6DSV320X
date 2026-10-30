#!/usr/bin/env python3
"""Receive and display CSV telemetry sent by the ESP32."""

import socket, csv, re, time


UDP_PORT = 5000
FIELDS = (
    "sequence,timestamp_us,accel_x_ms2,accel_y_ms2,accel_z_ms2,"
    "gyro_x_dps,gyro_y_dps,gyro_z_dps,pitch_deg,roll_deg,yaw_deg"
)
header = ["sequence", "timestamp_us", "accel_x_ms2", "accel_y_ms2", "accel_z_ms2", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "pitch_deg", "roll_deg", "yaw_deg"]
packets = []

def main() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("0.0.0.0", UDP_PORT))
        print(f"Listening for ESP32 telemetry on UDP port {UDP_PORT}")
        print(FIELDS)
        while True:
            
            packet, sender = sock.recvfrom(2048)
            data = packet.decode('utf-8').strip()
            #p = re.compile(r"\b(\d+)")
            #x = p.match(data)
            packet = data.split(",")
            print(f"{sender[0]}: {data}")
            #print(f"{sender[0]}: {x.group(0)}")
            #packets.append([x.group(0)])
            packets.append(packet)



if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        with open("sequences.csv", "w", newline='', encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(packets)
        print("\nReceiver stopped")
