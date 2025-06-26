#!/usr/bin/env python3
"""
Virtual Arduino Device Simulator for Hydrus
Creates a virtual serial port that mimics Arduino behavior when no real Arduino is connected.
"""

import os
import pty
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import serial


class VirtualArduinoDevice:
    def __init__(self, device_path="/dev/ttyACM0", baudrate=115200):
        self.device_path = device_path
        self.baudrate = baudrate
        self.master_fd = None
        self.slave_fd = None
        self.running = False
        self.thread = None

    def create_virtual_device(self):
        """Create a virtual serial device using socat"""
        try:
            # Check if device already exists
            if os.path.exists(self.device_path):
                print(f"Device {self.device_path} already exists")
                return True

            # Create virtual serial port using socat
            print(f"Creating virtual Arduino device at {self.device_path}")

            # Create the directory if it doesn't exist
            device_dir = os.path.dirname(self.device_path)
            os.makedirs(device_dir, exist_ok=True)

            # Use socat to create virtual serial port
            socat_cmd = [
                "socat",
                f"pty,link={self.device_path},raw,echo=0",
                f"pty,raw,echo=0",
            ]

            # Start socat in background
            self.socat_process = subprocess.Popen(
                socat_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )

            # Wait a bit for the device to be created
            time.sleep(2)

            # Set proper permissions
            os.chmod(self.device_path, 0o666)

            print(f"Virtual Arduino device created at {self.device_path}")
            return True

        except Exception as e:
            print(f"Failed to create virtual device: {e}")
            return False

    def simulate_arduino_responses(self):
        """Simulate Arduino responses to serial commands"""
        try:
            # Open the virtual serial port
            ser = serial.Serial(self.device_path, self.baudrate, timeout=1)
            print(f"Virtual Arduino listening on {self.device_path}")

            while self.running:
                try:
                    # Read incoming data
                    if ser.in_waiting > 0:
                        data = ser.readline().decode("utf-8").strip()
                        if data:
                            print(f"Virtual Arduino received: {data}")

                            # Simulate Arduino responses
                            if data.startswith("THRUSTER"):
                                # Respond to thruster commands
                                response = f"ACK:{data}\n"
                                ser.write(response.encode("utf-8"))
                                print(f"Virtual Arduino sent: {response.strip()}")

                            elif data == "STATUS":
                                # Respond to status requests
                                response = "STATUS:OK,THRUSTERS:8,DEPTH:0.0\n"
                                ser.write(response.encode("utf-8"))
                                print(f"Virtual Arduino sent: {response.strip()}")

                            elif data == "PING":
                                # Respond to ping
                                response = "PONG\n"
                                ser.write(response.encode("utf-8"))
                                print(f"Virtual Arduino sent: {response.strip()}")

                            else:
                                # Generic acknowledgment
                                response = f"OK:{data}\n"
                                ser.write(response.encode("utf-8"))
                                print(f"Virtual Arduino sent: {response.strip()}")

                    time.sleep(0.1)

                except serial.SerialException as e:
                    print(f"Serial error: {e}")
                    break
                except Exception as e:
                    print(f"Error in Arduino simulation: {e}")
                    break

            ser.close()

        except Exception as e:
            print(f"Failed to open virtual serial port: {e}")

    def start(self):
        """Start the virtual Arduino device"""
        if self.create_virtual_device():
            self.running = True
            self.thread = threading.Thread(target=self.simulate_arduino_responses)
            self.thread.daemon = True
            self.thread.start()
            return True
        return False

    def stop(self):
        """Stop the virtual Arduino device"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=5)

        # Clean up socat process
        if hasattr(self, "socat_process"):
            try:
                os.killpg(os.getpgid(self.socat_process.pid), signal.SIGTERM)
                self.socat_process.wait(timeout=5)
            except Exception as e:
                print(f"Error stopping socat: {e}")

        # Remove the virtual device file
        try:
            if os.path.exists(self.device_path):
                os.unlink(self.device_path)
                print(f"Removed virtual device {self.device_path}")
        except Exception as e:
            print(f"Error removing virtual device: {e}")


def check_and_create_virtual_arduino(device_path="/dev/ttyACM0"):
    """Check if Arduino exists, if not create a virtual one"""

    # First, check if real Arduino exists
    if os.path.exists(device_path):
        try:
            # Try to open it to see if it's accessible
            with serial.Serial(device_path, 115200, timeout=1):
                print(f"Real Arduino detected at {device_path}")
                return None  # Real Arduino exists, no need for virtual
        except Exception:
            print(f"Device {device_path} exists but is not accessible")

    print(f"No accessible Arduino found at {device_path}, creating virtual device...")

    # Install socat if not available
    try:
        subprocess.run(["which", "socat"], check=True, capture_output=True)
    except subprocess.CalledProcessError:
        print("Installing socat...")
        subprocess.run(["apt-get", "update"], check=True)
        subprocess.run(["apt-get", "install", "-y", "socat"], check=True)

    # Create and start virtual Arduino
    virtual_arduino = VirtualArduinoDevice(device_path)
    if virtual_arduino.start():
        print(f"Virtual Arduino started successfully at {device_path}")
        return virtual_arduino
    else:
        print("Failed to start virtual Arduino")
        return None


def signal_handler(signum, frame):
    """Handle shutdown signals"""
    print("\nShutting down virtual Arduino...")
    if hasattr(signal_handler, "virtual_arduino") and signal_handler.virtual_arduino:
        signal_handler.virtual_arduino.stop()
    sys.exit(0)


if __name__ == "__main__":
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Check command line arguments
    device_path = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"

    # Create virtual Arduino if needed
    virtual_arduino = check_and_create_virtual_arduino(device_path)
    signal_handler.virtual_arduino = virtual_arduino

    if virtual_arduino:
        print(f"Virtual Arduino running. Press Ctrl+C to stop.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            virtual_arduino.stop()
    else:
        print("Using real Arduino device")
