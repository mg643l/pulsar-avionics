import argparse
from datetime import datetime
import struct
import sys
import time
import traceback

import pigpio
from nrf24 import *

TELEMETRY_STRUCT = "<Bffff"  # Protocol, program_time, vertical_speed, altitude, phase
GPS_STRUCT = "<Bffff"        # Protocol, latitude, longitude, speed, course

def print_telemetry(payload):
    try:
        protocol, program_time, vertical_speed, altitude, phase = struct.unpack(TELEMETRY_STRUCT, payload)
        print(f"[TELEMETRY] Time: {program_time:.2f}s | VSpeed: {vertical_speed:.2f} m/s | Alt: {altitude:.2f} m | Phase: {phase:.0f}")
    except Exception as e:
        print(f"Error unpacking telemetry: {e}")

def print_gps(payload):
    try:
        protocol, latitude, longitude, speed, course = struct.unpack(GPS_STRUCT, payload)
        print(f"[GPS] Lat: {latitude:.6f} | Lon: {longitude:.6f} | Speed: {speed:.2f} knots | Course: {course:.2f}°")
    except Exception as e:
        print(f"Error unpacking GPS: {e}")

if __name__ == "__main__":

    print("Python NRF24 Telemetry Receiver")
    
    parser = argparse.ArgumentParser(prog="ground_station.py", description="NRF24 Telemetry Receiver")
    parser.add_argument('-n', '--hostname', type=str, default='localhost', help="Hostname for the Raspberry running the pigpio daemon.")
    parser.add_argument('-p', '--port', type=int, default=8888, help="Port number of the pigpio daemon.")
    parser.add_argument('address', type=str, nargs='?', default='1SNSR', help="Address to listen to (3 to 5 ASCII characters)")

    args = parser.parse_args()
    hostname = args.hostname
    port = args.port
    address = args.address

    if not (2 < len(address) < 6):
        print(f'Invalid address {address}. Addresses must be between 3 and 5 ASCII characters.')
        sys.exit(1)
    
    print(f'Connecting to GPIO daemon on {hostname}:{port} ...')
    pi = pigpio.pi(hostname, port)
    if not pi.connected:
        print("Not connected to Raspberry Pi ... goodbye.")
        sys.exit()

    nrf = NRF24(pi, ce=25, payload_size=RF24_PAYLOAD.DYNAMIC, channel=100, data_rate=RF24_DATA_RATE.RATE_250KBPS, pa_level=RF24_PA.MIN)
    nrf.set_address_bytes(len(address))
    nrf.open_reading_pipe(RF24_RX_ADDR.P1, address)
    nrf.show_registers()

    try:
        print(f'Receiving telemetry from {address}')
        count = 0
        while True:
            while nrf.data_ready():
                count += 1
                now = datetime.now()
                pipe = nrf.data_pipe()
                payload = nrf.get_payload()
                hex_data = ':'.join(f'{i:02x}' for i in payload)
                print(f"{now:%Y-%m-%d %H:%M:%S.%f}: pipe: {pipe}, len: {len(payload)}, bytes: {hex_data}, count: {count}")

                if len(payload) == 17 and payload[0] == 0x10:
                    print_telemetry(payload)
                elif len(payload) == 17 and payload[0] == 0x11:
                    print_gps(payload)
                else:
                    print("Unknown or malformed packet.")

            time.sleep(0.1)
    except:
        traceback.print_exc()
        nrf.power_down()
        pi.stop()
