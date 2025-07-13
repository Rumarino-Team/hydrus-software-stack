#!/usr/bin/env python3
"""
Arduino Debug Terminal using SOCAT virtual ports
Manual testing without stopping ROS systems
"""

import datetime
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import serial
from termcolor import colored


class Arduino:
    def __init__(self, arduino_device="/dev/ttyACM0"):
        self.arduino_device = arduino_device
        self.control_port = "/dev/hydrus_control"
        self.monitor_port = "/dev/hydrus_monitor"
        self.debug_port = "/dev/hydrus_debug"
        self.processes = {}
        self.running = False

        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle termination signals"""
        print(colored(f"Received signal {signum}, stopping multiplexer...", "red"))
        self._stop_multiplexer()

    def _stop_multiplexer(self):
        """Stop the multiplexer"""
        self.running = False

        # Kill all processes
        for name, process in self.processes.items():
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
            except Exception as e:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except Exception:
                    pass

        # Clean up virtual ports
        for port in [self.control_port, self.monitor_port, self.debug_port]:
            try:
                if os.path.exists(port):
                    os.unlink(port)
            except Exception as e:
                pass

    def _check_arduino_device(self):
        """Check if Arduino device exists"""
        if not os.path.exists(self.arduino_device):
            # Use virtual Arduino as fallback
            virtual_script = Path(__file__).parent / "virtual_arduino.py"
            if virtual_script.exists():
                subprocess.run(
                    [sys.executable, str(virtual_script), self.arduino_device]
                )
                time.sleep(3)
            return os.path.exists(self.arduino_device)
        return True

    def start_multiplexer(self):
        """Start simple Arduino multiplexer"""

        # Check Arduino device
        if not self._check_arduino_device():
            return False

        # Clean up existing ports
        for port in [self.control_port, self.monitor_port, self.debug_port]:
            if os.path.exists(port):
                os.unlink(port)

        try:
            port_configs = {
                "control": [
                    "pty,link={port},raw,echo=0",
                    "{device},raw,echo=0,b115200",
                ],
                "debug": [
                    "pty,link={port},raw,echo=0",
                    "{device},raw,echo=0,b115200",
                ],
                "monitor": [
                    "pty,link={port},raw,echo=0",
                    "EXEC:'cat {device}'",
                ],
            }

            for name, config in port_configs.items():
                port = getattr(self, f"{name}_port")
                cmd = [
                    "socat",
                    config[0].format(port=port),
                    config[1].format(device=self.arduino_device),
                ]

                self.processes[name] = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid,
                )

                time.sleep(2)

                # Verify port was created
                if not os.path.exists(port):
                    if name == "monitor":
                        continue  # Monitor port is optional
                    raise RuntimeError(f"Failed to create {name} port: {port}")

            self.running = True
            return True

        except Exception as e:
            self._stop_multiplexer()
            return False

    # -----------------------------------------------------# Arduino Debug Terminal Class
    # STATIC METHODS FOR ARDUINO DEBUG TERMINAL
    @staticmethod
    def arduino_debug_terminal(debug_port="/dev/hydrus_debug"):
        """Start interactive debug terminal (send-only)"""
        print(colored("Arduino Debug Terminal", "cyan", attrs=["bold"]))
        print(colored("Send-only mode - Commands will be sent to Arduino", "yellow"))
        print(colored("Type 'quit' to exit", "yellow"))
        print("=" * 50)

        # Wait for debug port to be available
        while not os.path.exists(debug_port):
            print(f"Waiting for debug port: {debug_port}")
            time.sleep(1)

        try:
            # Open debug port for sending commands only
            debug_ser = serial.Serial(debug_port, 115200, timeout=1)

            print(
                colored("✓ Connected to Arduino debug interface (send-only)", "green")
            )
            print(colored("ℹ️  Commands will be sent directly to Arduino", "blue"))
            print(colored("ℹ️  Use monitor scripts to see Arduino responses", "blue"))

            # Start send-only command loop
            while True:
                try:
                    # Get user input
                    cmd = input(colored(">> ", "blue")).strip()

                    if cmd.lower() in ["quit", "exit", "q"]:
                        break

                    if cmd:
                        # Send command to Arduino via debug port
                        debug_ser.write(f"{cmd}\n".encode("utf-8"))
                        debug_ser.flush()
                        print(colored(f">> {cmd} (sent)", "blue"))

                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(colored(f"Input error: {e}", "red"))

        except Exception as e:
            print(colored(f"Failed to connect: {e}", "red"))

        finally:
            print(colored("\nExiting debug terminal...", "yellow"))

    @staticmethod
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
                                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[
                                        :-3
                                    ]
                                    print(f"[{colored(timestamp, 'blue')}] {line}")
                        except Exception as e:
                            print(colored(f"Monitor error: {e}", "red"))
                            break

                        time.sleep(0.01)

            except Exception as e:
                print(colored(f"Failed to connect to monitor port: {e}", "red"))


def main():
    """Main function to run the Arduino debug terminal"""
    if len(sys.argv) > 1 and sys.argv[1] == "debug":
        Arduino.arduino_debug_terminal()
    elif len(sys.argv) > 1 and sys.argv[1] == "monitor":
        Arduino.arduino_monitor()
    elif len(sys.argv) > 1 and sys.argv[1] == "multiplexer":
        if len(sys.argv) > 2:
            arduino_device = sys.argv[2]
        else:
            arduino_device = "/dev/ttyACM0"

        arduino = Arduino(arduino_device)
        if arduino.start_multiplexer():
            print(colored("Arduino multiplexer started successfully!", "green"))
        else:
            print(colored("Failed to start Arduino multiplexer.", "red"))


if __name__ == "__main__":
    main()
