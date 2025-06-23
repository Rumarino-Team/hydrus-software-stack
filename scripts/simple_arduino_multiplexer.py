#!/usr/bin/env python3
"""
Simple Arduino Multiplexer - Minimal SOCAT Implementation
Creates basic virtual serial ports for Arduino access
"""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path


class SimpleArduinoMultiplexer:
    def __init__(self, arduino_device="/dev/ttyACM0"):
        self.arduino_device = arduino_device
        self.control_port = "/dev/hydrus_control"
        self.monitor_port = "/dev/hydrus_monitor"
        self.processes = {}
        self.running = False

        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _check_arduino_device(self):
        """Check if Arduino device exists"""
        if not os.path.exists(self.arduino_device):
            print(f"Warning: Arduino device {self.arduino_device} not found")
            print("Creating virtual Arduino...")
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
        print("Starting Simple Arduino Multiplexer...")

        # Check Arduino device
        if not self._check_arduino_device():
            print("Failed to setup Arduino device")
            return False

        # Clean up existing ports
        for port in [self.control_port, self.monitor_port]:
            if os.path.exists(port):
                os.unlink(port)

        try:
            # Create control port (bidirectional for ROS bridge)
            print(f"Creating control port: {self.control_port}")
            control_cmd = [
                "socat",
                f"pty,link={self.control_port},raw,echo=0",
                f"{self.arduino_device},raw,echo=0,b115200",
            ]

            self.processes["control"] = subprocess.Popen(
                control_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )

            time.sleep(2)

            # Verify control port was created
            if not os.path.exists(self.control_port):
                raise RuntimeError(
                    f"Failed to create control port: {self.control_port}"
                )

            print(f"✓ Control port created: {self.control_port}")

            # Create monitor port (read-only copy of Arduino data)
            print(f"Creating monitor port: {self.monitor_port}")
            monitor_cmd = [
                "socat",
                f"pty,link={self.monitor_port},raw,echo=0",
                f"EXEC:'cat {self.arduino_device}'",
            ]

            self.processes["monitor"] = subprocess.Popen(
                monitor_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )

            time.sleep(2)

            # Verify monitor port was created
            if not os.path.exists(self.monitor_port):
                print(f"Warning: Failed to create monitor port: {self.monitor_port}")
            else:
                print(f"✓ Monitor port created: {self.monitor_port}")

            self.running = True
            print("✓ Simple Arduino multiplexer started successfully")
            return True

        except Exception as e:
            print(f"Failed to start multiplexer: {e}")
            self.stop_multiplexer()
            return False

    def stop_multiplexer(self):
        """Stop the multiplexer"""
        print("Stopping Simple Arduino Multiplexer...")
        self.running = False

        # Kill all processes
        for name, process in self.processes.items():
            try:
                print(f"Stopping {name} process...")
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
            except Exception as e:
                print(f"Error stopping {name}: {e}")
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except Exception:
                    pass

        # Clean up virtual ports
        for port in [self.control_port, self.monitor_port]:
            try:
                if os.path.exists(port):
                    os.unlink(port)
                    print(f"✓ Removed {port}")
            except Exception as e:
                print(f"Error removing {port}: {e}")

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print(f"\nReceived signal {signum}, shutting down...")
        self.stop_multiplexer()
        sys.exit(0)

    def run_daemon(self):
        """Run multiplexer as daemon"""
        if not self.start_multiplexer():
            sys.exit(1)

        try:
            print("Multiplexer running. Press Ctrl+C to stop...")
            while self.running:
                # Monitor process health
                for name, process in list(self.processes.items()):
                    if process.poll() is not None:
                        print(f"Warning: {name} process died")
                        # Could implement restart logic here

                time.sleep(5)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_multiplexer()


def main():
    if len(sys.argv) > 1:
        arduino_device = sys.argv[1]
    else:
        arduino_device = "/dev/ttyACM0"

    multiplexer = SimpleArduinoMultiplexer(arduino_device)
    multiplexer.run_daemon()


if __name__ == "__main__":
    main()
