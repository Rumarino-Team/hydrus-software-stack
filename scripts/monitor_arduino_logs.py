#!/usr/bin/env python3
"""
Arduino Monitor using SOCAT virtual ports
Real-time monitoring without interfering with ROS bridge
"""

import os
import time
from datetime import datetime

import serial
from termcolor import colored


def arduino_monitor(monitor_port="/dev/hydrus_monitor"):
    """Start real-time Arduino monitoring"""
    print(colored("Arduino Monitor Starting...", "cyan"))
    print(colored(f"Monitoring port: {monitor_port}", "yellow"))
    print(colored("Press Ctrl+C to stop", "yellow"))
    print("=" * 60)

    while not os.path.exists(monitor_port):
        print(colored("Waiting for monitor port to be available...", "yellow"))
        time.sleep(1)

    try:
        with serial.Serial(monitor_port, 115200, timeout=1) as ser:
            print(colored("✓ Connected to Arduino monitor", "green"))

            while True:
                try:
                    if ser.in_waiting > 0:
                        line = ser.readline().decode("utf-8").strip()
                        if line:
                            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            print(f"[{colored(timestamp, 'blue')}] {line}")
                except Exception as e:
                    print(colored(f"Monitor error: {e}", "red"))
                    break

                time.sleep(0.01)

    except Exception as e:
        print(colored(f"Failed to connect to monitor port: {e}", "red"))


if __name__ == "__main__":
    try:
        arduino_monitor()
    except KeyboardInterrupt:
        print(colored("\nStopping Arduino monitor...", "yellow"))
