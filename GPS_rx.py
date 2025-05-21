import argparse
from datetime import datetime
import struct
import sys
import time
import traceback

import pigpio
from nrf24 import *

if __name__ == "__main__":
    print("Python NRF24 GPS Receiver Example.")
    
    parser = argparse.ArgumentParser(prog="gps-receiver.py", description="NRF24 GPS Receiver.")
    parser.add_argument('-n', '--hostname', type=str, default='localhost', help="Hostname for the Raspberry running the pigpio daemon.")
    parser.add_argument('-p', '--port', type=int, default=8888, help="Port number of the pigpio daemon.")
    parser.add_argument('address', type=str, nargs='?', default='1SNSR', help="Address to listen to (5 ASCII characters)")
    
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

    nrf = NRF24(pi, ce=17, payload_size=RF24_PAYLOAD.DYNAMIC, channel=100, data_rate=RF24_DATA_RATE.RATE_250KBPS, pa_level=RF24_PA.MIN)
    nrf.set_address_bytes(len(address))
    nrf.open_reading_pipe(RF24_RX_ADDR.P1, address)
    
    nrf.show_registers()

    try:
        print(f'Receiving data from {address}')
        count = 0
        while True:
            while nrf.data_ready():
                count += 1
                now = datetime.now()
                pipe = nrf.data_pipe()
                payload = nrf.get_payload()
                hex_data = ':'.join(f'{i:02x}' for i in payload)
                print(f"{now:%Y-%m-%d %H:%M:%S.%f}: pipe: {pipe}, len: {len(payload)}, bytes: {hex_data}, count: {count}")
                
                if len(payload) == 9 and payload[0] == 0x02:
                    try:
                        values = struct.unpack("<Bff", payload)
                        print(f'Protocol: {values[0]}, Latitude: {values[1]}, Longitude: {values[2]}')
                    except struct.error:
                        print("Error unpacking received data.")
            
            time.sleep(0.1)
    except:
        traceback.print_exc()
        nrf.power_down()
        pi.stop()
