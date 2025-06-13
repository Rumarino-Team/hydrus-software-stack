#!/usr/bin/env python3
"""
Hydrus Arduino Log Monitor
Replaces monitor_arduino_logs.sh with improved error handling
"""

import time
from pathlib import Path


class ArduinoLogMonitor:
    def __init__(self):
        self.log_file = Path("/tmp/hydrus_serial/arduinolog.txt")

    def wait_for_log_file(self):
        """Wait for the Arduino log file to be created"""
        print("Waiting for Arduino log file...")
        while not self.log_file.exists():
            time.sleep(1)
            print(".", end="", flush=True)
        print()

    def monitor_logs(self):
        """Monitor and display Arduino logs"""
        print("Found Arduino log file. Showing Arduino serial output:")
        print("------------------------------------------------------")

        try:
            # Use tail -f equivalent in Python
            with open(self.log_file, "r") as f:
                # Go to end of file
                f.seek(0, 2)

                while True:
                    line = f.readline()
                    if line:
                        print(line.rstrip())
                    else:
                        time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopping Arduino log monitor...")
        except Exception as e:
            print(f"Error monitoring logs: {e}")

    def main(self):
        """Main execution function"""
        self.wait_for_log_file()
        self.monitor_logs()


if __name__ == "__main__":
    monitor = ArduinoLogMonitor()
    monitor.main()
