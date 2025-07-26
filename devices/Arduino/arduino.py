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

# Import debug utilities for enhanced debugging
try:
    from scripts.debug_utils import (
        SmartDebugger,
        breakpoint_here,
        conditional_breakpoint,
        debug_method,
        debug_with_env_check,
        start_debug_server,
    )

    DEBUG_UTILS_AVAILABLE = True
except ImportError:
    print(
        colored(
            "Debug utils not available, continuing without enhanced debugging", "yellow"
        )
    )
    DEBUG_UTILS_AVAILABLE = False

    # Create dummy functions to avoid errors
    def debug_with_env_check(*args, **kwargs):
        return False

    def debug_method(*args, **kwargs):
        def decorator(func):
            return func

        return decorator

    def start_debug_server(*args, **kwargs):
        return False

    def breakpoint_here(*args, **kwargs):
        pass

    def conditional_breakpoint(*args, **kwargs):
        pass

    class SmartDebugger:
        def __init__(self, *args, **kwargs):
            pass

        def start_session(self, *args, **kwargs):
            pass

        def breakpoint(self, *args, **kwargs):
            pass

        def end_session(self):
            pass


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

        # Initialize debugger if available
        if DEBUG_UTILS_AVAILABLE:
            self.debugger = SmartDebugger("arduino", port=5679)
            # Check if Arduino debugging is enabled
            self._debug_enabled = debug_with_env_check("ARDUINO_DEBUG")
            if self._debug_enabled:
                self.debugger.start_session("Arduino debugging enabled")
        else:
            self.debugger = None
            self._debug_enabled = False

    def _signal_handler(self, signum, frame):
        """Handle termination signals"""
        print(colored(f"\nReceived signal {signum}, stopping multiplexer...", "red"))
        self._stop_multiplexer()
        sys.exit(0)

    def _stop_multiplexer(self):
        """Stop the multiplexer"""
        print(colored("Stopping Arduino multiplexer...", "yellow"))
        self.running = False

        # Kill all processes
        for name, process in self.processes.items():
            try:
                print(colored(f"Stopping {name} process...", "yellow"))
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
            except Exception as e:
                print(colored(f"Error stopping {name}: {e}", "red"))
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except Exception:
                    pass

        # Clean up virtual ports
        for port in [self.control_port, self.monitor_port, self.debug_port]:
            try:
                if os.path.exists(port):
                    os.unlink(port)
                    print(colored(f"✓ Removed {port}", "green"))
            except Exception as e:
                print(colored(f"Error removing {port}: {e}", "red"))

    @debug_method(wait=False)
    def _check_arduino_device(self):
        """Check if Arduino device exists"""
        if self._debug_enabled:
            breakpoint_here(f"Checking Arduino device: {self.arduino_device}")

        if not os.path.exists(self.arduino_device):
            print(
                colored(
                    f"Warning: Arduino device {self.arduino_device} not found", "yellow"
                )
            )
            print(colored("Creating virtual Arduino...", "cyan"))
            # Use virtual Arduino as fallback
            virtual_script = Path(__file__).parent / "virtual_arduino.py"
            if virtual_script.exists():
                subprocess.run(
                    [sys.executable, str(virtual_script), self.arduino_device]
                )
                time.sleep(3)
            return os.path.exists(self.arduino_device)
        return True

    @debug_method(wait=False)
    def start_multiplexer(self):
        """Start simple Arduino multiplexer"""
        print(colored("Starting Arduino Multiplexer...", "cyan"))

        if self._debug_enabled:
            self.debugger.breakpoint("Starting multiplexer setup")

        # Check Arduino device
        if not self._check_arduino_device():
            print(colored("Failed to setup Arduino device", "red"))
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

                if self._debug_enabled:
                    conditional_breakpoint(
                        name == "debug", f"Setting up debug port: {port}"
                    )

                print(colored(f"Creating {name} port: {port}", "cyan"))

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
                        print(
                            colored(
                                f"Warning: Failed to create {name} port: {port}",
                                "yellow",
                            )
                        )
                        continue  # Monitor port is optional
                    raise RuntimeError(f"Failed to create {name} port: {port}")
                else:
                    print(
                        colored(f"✓ {name.capitalize()} port created: {port}", "green")
                    )

            self.running = True
            print(colored("✓ Arduino multiplexer started successfully", "green"))
            print("=" * 60)
            print(colored("Available ports:", "cyan"))
            print(
                colored(f"  Control Port:  {self.control_port} (ROS Bridge)", "white")
            )
            print(
                colored(
                    f"  Debug Port:    {self.debug_port} (Interactive Debug)", "white"
                )
            )
            if os.path.exists(self.monitor_port):
                print(
                    colored(
                        f"  Monitor Port:  {self.monitor_port} (Read-only Monitor)",
                        "white",
                    )
                )
            print("=" * 60)
            return True

        except Exception as e:
            print(colored(f"Failed to start multiplexer: {e}", "red"))
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

        # Wait for monitor port to be available
        max_wait_time = 30  # Maximum wait time in seconds
        wait_time = 0
        while not os.path.exists(monitor_port) and wait_time < max_wait_time:
            print(colored("Waiting for monitor port to be available...", "yellow"))
            time.sleep(1)
            wait_time += 1

        if not os.path.exists(monitor_port):
            print(
                colored(
                    f"Monitor port {monitor_port} not available after {max_wait_time}s",
                    "red",
                )
            )
            print(
                colored("Make sure the Arduino multiplexer is running first", "yellow")
            )
            return

        # Connect to the monitor port
        try:
            print(colored("Attempting to connect to monitor port...", "cyan"))
            with serial.Serial(monitor_port, 115200, timeout=1) as ser:
                print(colored("✓ Connected to Arduino monitor", "green"))
                print(
                    colored("Monitoring Arduino output (press Ctrl+C to stop):", "cyan")
                )
                print("-" * 60)

                try:
                    while True:
                        try:
                            if ser.in_waiting > 0:
                                line = (
                                    ser.readline()
                                    .decode("utf-8", errors="ignore")
                                    .strip()
                                )
                                if line:
                                    timestamp = datetime.datetime.now().strftime(
                                        "%H:%M:%S.%f"
                                    )[:-3]
                                    print(f"[{colored(timestamp, 'blue')}] {line}")

                            # Check if port still exists
                            if not os.path.exists(monitor_port):
                                print(colored("Monitor port disappeared!", "red"))
                                break

                        except serial.SerialException as e:
                            print(colored(f"Serial connection lost: {e}", "red"))
                            break
                        except Exception as e:
                            print(colored(f"Monitor error: {e}", "red"))
                            break

                        time.sleep(0.01)

                except KeyboardInterrupt:
                    print(colored("\n✓ Monitoring stopped by user", "yellow"))

        except KeyboardInterrupt:
            print(colored("\n✓ Monitoring stopped by user", "yellow"))
        except Exception as e:
            print(colored(f"Failed to connect to monitor port: {e}", "red"))
            print(colored("Troubleshooting tips:", "yellow"))
            print(colored("  1. Check if Arduino multiplexer is running", "yellow"))
            print(colored("  2. Verify Arduino is connected", "yellow"))
            print(colored("  3. Try restarting the multiplexer", "yellow"))

    @staticmethod
    def start_debug_server(port=5678):
        """Start debugpy server for remote debugging"""
        if DEBUG_UTILS_AVAILABLE:
            # Use enhanced debug utilities
            return start_debug_server(port=port, wait_for_client=True)
        else:
            # Fallback to basic implementation
            try:
                import debugpy

                debugpy.listen(("0.0.0.0", port))
                print(colored(f"🐛 Debug server started on port {port}", "cyan"))
                print(colored("Waiting for debugger to attach...", "yellow"))
                debugpy.wait_for_client()
                print(colored("✓ Debugger attached!", "green"))
                return True
            except ImportError:
                print(
                    colored(
                        "debugpy not installed. Install with: pip install debugpy",
                        "red",
                    )
                )
                return False
            except Exception as e:
                print(colored(f"Failed to start debug server: {e}", "red"))
                return False

    def run_daemon(self):
        """Run multiplexer as daemon with health monitoring"""
        if not self.start_multiplexer():
            print(colored("Failed to start multiplexer", "red"))
            sys.exit(1)

        try:
            print(
                colored(
                    "Multiplexer running as daemon. Press Ctrl+C to stop...", "green"
                )
            )
            while self.running:
                # Monitor process health
                for name, process in list(self.processes.items()):
                    if process.poll() is not None:
                        print(
                            colored(
                                f"Warning: {name} process died (exit code: {process.returncode})",
                                "yellow",
                            )
                        )
                        # Remove dead process from tracking
                        del self.processes[name]

                        # Check if critical processes died
                        if name == "control":
                            print(
                                colored(
                                    "Critical control process died, restarting multiplexer...",
                                    "red",
                                )
                            )
                            self._stop_multiplexer()
                            time.sleep(2)
                            if not self.start_multiplexer():
                                print(colored("Failed to restart multiplexer", "red"))
                                sys.exit(1)
                            break

                time.sleep(5)
        except KeyboardInterrupt:
            print(colored("\nShutdown requested by user", "yellow"))
        finally:
            self._stop_multiplexer()

    def stop_daemon(self):
        """Stop the daemon"""
        self.running = False
        self._stop_multiplexer()


def main():
    """Main function to run the Arduino debug terminal"""
    # Enhanced debug support using debug utilities
    if DEBUG_UTILS_AVAILABLE:
        # Check for various debug environment variables
        debug_with_env_check("ARDUINO_DEBUG")

        # Check command line arguments for debug flags
        if "--debug" in sys.argv:
            print(colored("🐛 Debug mode enabled via --debug flag", "cyan"))
            Arduino.start_debug_server()
            sys.argv.remove("--debug")

        # Support additional debug options
        if "--debug-env" in sys.argv:
            print(colored("Available debug environment variables:", "cyan"))
            print(colored("  ARDUINO_DEBUG=1    - Enable Arduino debugging", "yellow"))
            print(colored("  DEBUG=1           - Enable general debugging", "yellow"))
            print(colored("  DEBUG_PORT=5678   - Set debug port", "yellow"))
            sys.argv.remove("--debug-env")
            return
    else:
        # Fallback for basic debug support
        if "--debug" in sys.argv:
            Arduino.start_debug_server()
            sys.argv.remove("--debug")

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

        # Always run in daemon mode with health monitoring
        arduino.run_daemon()


if __name__ == "__main__":
    main()
