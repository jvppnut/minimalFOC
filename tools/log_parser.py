#!/usr/bin/env python3
"""
minimalFOC binary log parser
Reads 47-byte frames from the STM32 UART and writes CSV to stdout.

Frame layout:
  [0xAA][0x55] | 11 x float32 LE (44 bytes) | XOR checksum (1 byte)

Fields (in order):
  theta_mech   rad    unwrapped rotor mechanical angle
  omega_mech   rad/s  rotor mechanical velocity (filtered)
  theta_elec   rad    electrical angle
  i_u          A      phase U current
  i_v          A      phase V current
  i_w          A      phase W current
  i_d          A      d-axis current
  i_q          A      q-axis current
  v_d          V      d-axis applied voltage
  v_q          V      q-axis applied voltage
  isr_us       us     ISR processing time

Usage:
  python log_parser.py COM3            # print CSV to console
  python log_parser.py COM3 > data.csv # save to file
  python log_parser.py /dev/ttyUSB0   # Linux/Mac
"""

import sys
import struct
import serial

PORT     = sys.argv[1] if len(sys.argv) > 1 else 'COM3'
BAUD     = 921600
HEADER   = bytes([0xAA, 0x55])
FRAME_SZ = 47
FIELDS   = [
    'theta_mech', 'omega_mech', 'theta_elec',
    'i_u', 'i_v', 'i_w',
    'i_d', 'i_q',
    'v_d', 'v_q',
    'isr_us'
]

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f'Opened {PORT} at {BAUD} baud', file=sys.stderr)

    print(','.join(FIELDS))

    buf      = bytearray()
    n_ok     = 0
    n_bad    = 0

    try:
        while True:
            chunk = ser.read(max(ser.in_waiting, 1))
            if not chunk:
                continue
            buf.extend(chunk)

            while len(buf) >= FRAME_SZ:
                idx = buf.find(HEADER)
                if idx < 0:
                    del buf[:]
                    break
                if idx:
                    del buf[:idx]
                if len(buf) < FRAME_SZ:
                    break

                frame = bytes(buf[:FRAME_SZ])

                crc = 0
                for b in frame[2:46]:
                    crc ^= b

                if crc != frame[46]:
                    del buf[:1]   # discard one byte and re-sync
                    n_bad += 1
                    continue

                values = struct.unpack_from('<11f', frame, 2)
                print(','.join(f'{v:.6g}' for v in values), flush=True)
                del buf[:FRAME_SZ]
                n_ok += 1

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print(f'\nFrames OK: {n_ok}  Bad CRC: {n_bad}', file=sys.stderr)


if __name__ == '__main__':
    main()
