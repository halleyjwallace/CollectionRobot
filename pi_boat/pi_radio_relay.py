#!/usr/bin/env python3
"""Boat-side relay: NRF24 radio <-> robot Arduino Uno (USB serial).

Runs on the Raspberry Pi Zero 2 W on the boat.

  shore bridge Uno --radio--> this Pi --USB--> robot Uno (four_thrusters.ino)
                   <--radio--        <--USB--

* Each radio packet is one command line ("LF,LR,RF,RR,pump" or "CAL");
  it is forwarded to the robot Uno with a newline added.
* Lines from the robot Uno go back to shore inside the radio
  acknowledgements: status lines as text ('T' + text), IMU lines packed as
  binary ('I' + 5 floats) so they fit in one 32-byte packet.
* Pi failsafe: if no packet arrives for PI_FAILSAFE_S, a stop command is
  sent to the Uno at once (the Uno's own 0.5 s failsafe stays as a backup).

Radio wiring (as the original pi_receiver.py): CE = GPIO22, CSN = CE0 (SPI0).
Needs: pip install pyrf24 pyserial   and SPI enabled (raspi-config).
"""
import struct
import sys
import time
from collections import deque

import serial
from pyrf24 import RF24, RF24_250KBPS, RF24_PA_LOW

SERIAL_PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = 115200
CE_PIN = 22
CSN_DEVICE = 0
ADDRESS = b"BOAT1"       # must match radio_bridge_shore.ino
CHANNEL = 108            # must match radio_bridge_shore.ino
PI_FAILSAFE_S = 0.3
STOP_LINE = b"1500,1500,1500,1500,0\n"


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def open_arduino():
    """Open the robot Uno, retrying until it is plugged in."""
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0)
            log(f"Opened {SERIAL_PORT}; waiting 3 s for the Uno to restart "
                "(keep the robot still - gyro calibration)")
            time.sleep(3)
            return ser
        except serial.SerialException as err:
            log(f"Cannot open {SERIAL_PORT} ({err}); retrying in 2 s")
            time.sleep(2)


def setup_radio():
    radio = RF24(CE_PIN, CSN_DEVICE)
    if not radio.begin():
        raise RuntimeError("NRF24 not responding - check wiring and SPI")
    radio.channel = CHANNEL
    radio.data_rate = RF24_250KBPS
    radio.pa_level = RF24_PA_LOW
    radio.dynamic_payloads = True
    radio.ack_payloads = True
    radio.open_rx_pipe(1, ADDRESS)
    radio.listen = True
    return radio


def encode_imu(text):
    """'IMU,yr,yaw,ax,ay,az' -> b'I' + 5 little-endian floats, or None."""
    try:
        values = [float(v) for v in text[4:].split(",")]
    except ValueError:
        return None
    if len(values) != 5:
        return None
    return b"I" + struct.pack("<5f", *values)


def encode_text(text):
    return b"T" + text.encode("ascii", "replace")[:31]


def main():
    radio = setup_radio()
    ser = open_arduino()

    rx = bytearray()
    statuses = deque(maxlen=20)   # status lines waiting to go to shore
    latest_imu = None             # newest IMU packet waiting to go to shore
    last_packet = 0.0
    link_up = False

    radio.write_ack_payload(1, encode_text("RELAY READY"))
    log("Relay running")

    while True:
        # 1) Robot Uno -> replies for shore
        try:
            waiting = ser.in_waiting
            if waiting:
                rx += ser.read(waiting)
        except (serial.SerialException, OSError) as err:
            log(f"Uno serial error ({err}); reopening")
            ser.close()
            ser = open_arduino()
            continue
        while b"\n" in rx:
            raw, _, rx = rx.partition(b"\n")
            text = raw.decode("ascii", "ignore").strip()
            if not text:
                continue
            if text.startswith("IMU,"):
                latest_imu = encode_imu(text) or latest_imu
            else:
                statuses.append(encode_text(text))
                log(f"Uno: {text}")

        # 2) Radio -> robot Uno
        while radio.available():
            size = radio.get_dynamic_payload_size()
            if size < 1 or size > 32:
                radio.flush_rx()
                break
            data = bytes(radio.read(size))

            # Load the reply that the NEXT packet's acknowledgement will carry
            reply = statuses.popleft() if statuses else latest_imu
            latest_imu = None if reply is latest_imu else latest_imu
            radio.flush_tx()
            if reply:
                radio.write_ack_payload(1, reply)

            command = data.decode("ascii", "ignore").strip()
            if not command:
                continue
            try:
                ser.write(command.encode("ascii") + b"\n")
            except (serial.SerialException, OSError) as err:
                log(f"Could not write to Uno ({err})")
            last_packet = time.monotonic()
            if not link_up:
                link_up = True
                log("Radio link up")

        # 3) Pi failsafe: radio silent -> stop now
        if link_up and time.monotonic() - last_packet > PI_FAILSAFE_S:
            link_up = False
            try:
                ser.write(STOP_LINE)
            except (serial.SerialException, OSError):
                pass
            log("Radio link lost -> STOP sent to Uno")

        time.sleep(0.002)


if __name__ == "__main__":
    main()
