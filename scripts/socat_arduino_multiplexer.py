#!/usr/bin/env python3
"""
SOCAT Arduino Multiplexer
Creates virtual serial ports for multiple Arduino access
"""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional


class ArduinoMultiplexer:
    def __init__(self, arduino_device="/dev/ttyACM0", baud_rate=115200):
        self.arduino_device = arduino_device
        self.baud_rate = baud_rate

        # Virtual device paths
        self.control_port = "/dev/hydrus_control"  # Read/Write for ROS bridge
        self.monitor_port = "/dev/hydrus_monitor"  # Read-only for monitoring
        self.debug_port = "/dev/hydrus_debug"  # Read-only for manual testing
        self.log_port = "/dev/hydrus_log"  # Read-only for file logging

        # Process management
        self.socat_processes: Dict[str, subprocess.Popen] = {}
        self.running = False

        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _check_dependencies(self):
        """Ensure SOCAT is installed"""
        try:
            subprocess.run(["socat", "-V"], capture_output=True, check=True)
            print("✓ SOCAT found")
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("Installing SOCAT...")
            subprocess.run(["apt-get", "update"], check=True)
            subprocess.run(["apt-get", "install", "-y", "socat"], check=True)

    def _check_arduino_device(self):
        """Verify Arduino device exists and is accessible"""
        if not os.path.exists(self.arduino_device):
            raise FileNotFoundError(f"Arduino device {self.arduino_device} not found")

        if not os.access(self.arduino_device, os.R_OK | os.W_OK):
            raise PermissionError(f"No read/write access to {self.arduino_device}")

    def _create_virtual_port(
        self, name: str, virtual_path: str, readonly: bool = False
    ) -> subprocess.Popen:
        """Create a single virtual serial port"""

        # Remove existing virtual device
        if os.path.exists(virtual_path):
            os.unlink(virtual_path)

        # Create virtual port with SOCAT
        if readonly:
            # Read-only virtual port
            socat_cmd = [
                "socat",
                f"pty,link={virtual_path},raw,echo=0,user=nobody,group=dialout,perm=0644",
                f"pipe:/tmp/hydrus_arduino_data",
            ]
        else:
            # Read/write virtual port (control port)
            socat_cmd = [
                "socat",
                f"pty,link={virtual_path},raw,echo=0,user=nobody,group=dialout,perm=0666",
                f"{self.arduino_device},raw,echo=0,b{self.baud_rate}",
            ]

        print(f"Creating {name} virtual port: {virtual_path}")
        process = subprocess.Popen(
            socat_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )

        return process

    def _create_simple_multiplexer(self):
        """Simple and reliable SOCAT multiplexer implementation"""

        # Remove existing virtual devices
        for port in [
            self.control_port,
            self.monitor_port,
            self.debug_port,
            self.log_port,
        ]:
            if os.path.exists(port):
                os.unlink(port)

        # 1. Create bidirectional control port (for ROS bridge)
        print(f"Creating control port: {self.control_port}")
        control_cmd = [
            "socat",
            f"pty,link={self.control_port},raw,echo=0",
            f"{self.arduino_device},raw,echo=0,b{self.baud_rate}",
        ]

        self.socat_processes["control"] = subprocess.Popen(
            control_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )

        time.sleep(2)  # Allow control port to stabilize

        # 2. Create read-only monitoring ports using tee
        print("Creating monitoring ports...")

        # Create named pipe for data distribution
        pipe_path = "/tmp/hydrus_arduino_pipe"
        if os.path.exists(pipe_path):
            os.unlink(pipe_path)
        os.mkfifo(pipe_path, 0o666)

        # Start Arduino reader that feeds the pipe
        reader_cmd = f"cat {self.arduino_device} > {pipe_path} &"
        subprocess.run(["bash", "-c", reader_cmd])

        # Create individual monitor ports
        for port_name, port_path in [
            ("monitor", self.monitor_port),
            ("debug", self.debug_port),
            ("log", self.log_port),
        ]:
            print(f"Creating {port_name} port: {port_path}")

            monitor_cmd = [
                "socat",
                f"pty,link={port_path},raw,echo=0",
                f"pipe:{pipe_path}",
            ]

            self.socat_processes[port_name] = subprocess.Popen(
                monitor_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )

            time.sleep(1)  # Allow each port to be created

    def start_multiplexer(self):
        """Start the complete multiplexer system"""
        print("Starting Arduino SOCAT Multiplexer...")

        try:
            # Check dependencies and device
            self._check_dependencies()
            self._check_arduino_device()

            # Method 1: Simple approach with single control port + data splitter
            self._start_simple_multiplexer()

            self.running = True
            print("✓ Arduino multiplexer started successfully")
            print(f"✓ Control port (RW): {self.control_port}")
            print(f"✓ Monitor port (RO): {self.monitor_port}")
            print(f"✓ Debug port (RO): {self.debug_port}")
            print(f"✓ Log port (RO): {self.log_port}")

        except Exception as e:
            print(f"✗ Failed to start multiplexer: {e}")
            self.stop_multiplexer()
            raise

    def _start_simple_multiplexer(self):
        """Simple multiplexer implementation"""

        # Use the simple and reliable multiplexer
        self._create_simple_multiplexer()

        # Verify all ports exist
        self._verify_ports()

    def _verify_ports(self):
        """Verify all virtual ports were created successfully"""
        ports = [self.control_port, self.monitor_port, self.debug_port, self.log_port]

        for port in ports:
            if not os.path.exists(port):
                raise RuntimeError(f"Failed to create virtual port: {port}")
            print(f"✓ Virtual port created: {port}")

    def stop_multiplexer(self):
        """Stop all SOCAT processes and cleanup"""
        print("Stopping Arduino multiplexer...")
        self.running = False

        # Kill all SOCAT processes
        for name, process in self.socat_processes.items():
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

        # Cleanup virtual device files
        ports = [self.control_port, self.monitor_port, self.debug_port, self.log_port]
        for port in ports:
            try:
                if os.path.exists(port):
                    os.unlink(port)
                    print(f"✓ Removed {port}")
            except Exception as e:
                print(f"Error removing {port}: {e}")

        # Cleanup named pipes
        try:
            pipe_paths = ["/tmp/hydrus_arduino_data", "/tmp/hydrus_arduino_pipe"]
            for pipe_path in pipe_paths:
                if os.path.exists(pipe_path):
                    os.unlink(pipe_path)
        except Exception:
            pass

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print(f"\nReceived signal {signum}, shutting down...")
        self.stop_multiplexer()
        sys.exit(0)

    def run_daemon(self):
        """Run multiplexer as daemon"""
        self.start_multiplexer()

        try:
            while self.running:
                # Monitor processes health
                for name, process in self.socat_processes.items():
                    if process.poll() is not None:
                        print(f"Warning: {name} process died, restarting...")
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

    multiplexer = ArduinoMultiplexer(arduino_device)
    multiplexer.run_daemon()


if __name__ == "__main__":
    main()
