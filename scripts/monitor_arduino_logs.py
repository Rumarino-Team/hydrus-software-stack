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


class ArduinoMonitor:
    def __init__(self, monitor_port="/dev/hydrus_monitor"):
        self.monitor_port = monitor_port
        self.running = False

    def start_monitoring(self):
        """Start real-time Arduino monitoring"""
        print(colored("Arduino Monitor Starting...", "cyan"))
        print(colored(f"Monitoring port: {self.monitor_port}", "yellow"))
        print(colored("Press Ctrl+C to stop", "yellow"))
        print("=" * 60)

        while not os.path.exists(self.monitor_port):
            print(colored("Waiting for monitor port to be available...", "yellow"))
            time.sleep(1)

        try:
            with serial.Serial(self.monitor_port, 115200, timeout=1) as ser:
                self.running = True
                print(colored("✓ Connected to Arduino monitor", "green"))

                while self.running:
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

    def stop_monitoring(self):
        """Stop monitoring"""
        self.running = False


def main():
    monitor = ArduinoMonitor()
    try:
        monitor.start_monitoring()
    except KeyboardInterrupt:
        print(colored("\nStopping Arduino monitor...", "yellow"))
        monitor.stop_monitoring()


if __name__ == "__main__":
    main()
