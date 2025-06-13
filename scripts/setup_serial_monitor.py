#!/usr/bin/env python3
"""
Hydrus Serial Monitor Setup
Replaces setup_serial_monitor.sh with improved error handling
"""

import os
import sys
import subprocess
import signal
import time
from pathlib import Path
from typing import List


class SerialMonitorSetup:
    def __init__(self):
        self.arduino_port = "/dev/ttyACM0"
        self.baud_rate = 115200
        self.serial_dir = Path("/tmp/hydrus_serial")
        self.log_file = self.serial_dir / "arduinolog.txt"
        self.pid_file = self.serial_dir / "catpid.txt"
        self.cat_process = None
    
    def _run_command(self, cmd: List[str], check: bool = True, capture_output: bool = False) -> subprocess.CompletedProcess:
        """Run a command with proper error handling"""
        try:
            return subprocess.run(cmd, check=check, capture_output=capture_output, text=True)
        except subprocess.CalledProcessError as e:
            if check:
                print(f"Command failed: {' '.join(cmd)}")
                print(f"Exit code: {e.returncode}")
            raise
    
    def _check_socat_installed(self):
        """Check if socat is installed and install if needed"""
        try:
            self._run_command(["socat", "-V"], capture_output=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("Installing socat...")
            self._run_command(["apt-get", "update"])
            self._run_command(["apt-get", "install", "-y", "socat"])
    
    def _setup_arduino_port(self):
        """Setup Arduino port with proper permissions and configuration"""
        arduino_path = Path(self.arduino_port)
        
        # Check if port exists and is writable
        if not arduino_path.exists():
            print(f"Error: Arduino port {self.arduino_port} does not exist")
            return False
        
        if not os.access(self.arduino_port, os.W_OK):
            print(f"Error: Cannot write to {self.arduino_port}")
            print("Make sure the port exists and you have permission to access it")
            return False
        
        # Configure serial port
        try:
            stty_cmd = [
                "stty", "-F", self.arduino_port, "raw", 
                f"speed", str(self.baud_rate), "cs8", "-cstopb", "-parenb"
            ]
            self._run_command(stty_cmd)
            print(f"Arduino port {self.arduino_port} configured successfully")
            return True
        except subprocess.CalledProcessError as e:
            print(f"Failed to configure Arduino port: {e}")
            return False
    
    def _setup_logging_directory(self):
        """Setup the logging directory and files"""
        print("Setting up pipes for serial communication")
        
        # Create directory
        self.serial_dir.mkdir(exist_ok=True)
        
        # Remove existing log file
        if self.log_file.exists():
            self.log_file.unlink()
    
    def _start_serial_logging(self):
        """Start background process to read from Arduino and log to file"""
        try:
            # Use tee to both display and log the serial data
            with open(self.log_file, 'w') as log_f:
                self.cat_process = subprocess.Popen([
                    "cat", self.arduino_port
                ], stdout=log_f, stderr=subprocess.DEVNULL)
            
            # Save the PID
            self.pid_file.write_text(str(self.cat_process.pid))
            
            print(f"Serial splitter running with PID: {self.cat_process.pid}")
            print(f"Arduino logs will be saved to {self.log_file}")
            
            return True
            
        except Exception as e:
            print(f"Failed to start serial logging: {e}")
            return False
    
    def _cleanup(self, signum=None, frame=None):
        """Cleanup processes on exit"""
        if self.cat_process:
            try:
                self.cat_process.terminate()
                self.cat_process.wait(timeout=5)
            except:
                try:
                    self.cat_process.kill()
                except:
                    pass
    
    def main(self):
        """Main execution function"""
        # Handle command line argument for different port
        if len(sys.argv) > 1:
            self.arduino_port = sys.argv[1]
        
        print("Creating virtual serial ports for Arduino monitoring...")
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._cleanup)
        signal.signal(signal.SIGTERM, self._cleanup)
        
        try:
            # Check and install socat if needed
            self._check_socat_installed()
            
            # Setup Arduino port
            if not self._setup_arduino_port():
                sys.exit(1)
            
            # Setup logging directory
            self._setup_logging_directory()
            
            # Start serial logging
            if not self._start_serial_logging():
                sys.exit(1)
            
            # Keep the process running
            try:
                while True:
                    if self.cat_process and self.cat_process.poll() is not None:
                        print("Serial logging process stopped unexpectedly")
                        break
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\nStopping serial monitor...")
        
        finally:
            self._cleanup()


if __name__ == "__main__":
    setup = SerialMonitorSetup()
    setup.main()