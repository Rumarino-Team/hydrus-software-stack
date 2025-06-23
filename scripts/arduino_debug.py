#!/usr/bin/env python3
"""
Arduino Debug Terminal using SOCAT virtual ports
Manual testing without stopping ROS systems
"""

import os
import sys
import threading
import time

import serial
from termcolor import colored


class ArduinoDebugTerminal:
    def __init__(
        self, debug_port="/dev/hydrus_debug", control_port="/dev/hydrus_control"
    ):
        self.debug_port = debug_port  # Read-only monitoring
        self.control_port = control_port  # Write access for commands
        self.running = False

    def start_terminal(self):
        """Start interactive debug terminal"""
        print(colored("Arduino Debug Terminal", "cyan", attrs=["bold"]))
        print(colored("Commands will be sent via control port", "yellow"))
        print(colored("Responses monitored via debug port", "yellow"))
        print(colored("Type 'quit' to exit", "yellow"))
        print("=" * 50)

        # Wait for ports to be available
        while not (
            os.path.exists(self.debug_port) and os.path.exists(self.control_port)
        ):
            print("Waiting for virtual ports...")
            time.sleep(1)

        try:
            # Open control port for sending commands
            control_ser = serial.Serial(self.control_port, 115200, timeout=1)

            # Open debug port for monitoring responses
            debug_ser = serial.Serial(self.debug_port, 115200, timeout=1)

            print(colored("✓ Connected to Arduino debug interface", "green"))

            # Start response monitoring thread
            self.running = True
            monitor_thread = threading.Thread(
                target=self._monitor_responses, args=(debug_ser,)
            )
            monitor_thread.daemon = True
            monitor_thread.start()

            # Main command input loop
            self._command_loop(control_ser)

        except Exception as e:
            print(colored(f"Failed to connect: {e}", "red"))
        finally:
            self.running = False

    def _monitor_responses(self, debug_ser):
        """Monitor Arduino responses in background thread"""
        while self.running:
            try:
                if debug_ser.in_waiting > 0:
                    line = debug_ser.readline().decode("utf-8").strip()
                    if line:
                        print(colored(f"<< {line}", "green"))
            except Exception as e:
                if self.running:
                    print(colored(f"Monitor error: {e}", "red"))
                break
            time.sleep(0.01)

    def _command_loop(self, control_ser):
        """Interactive command input loop"""
        while self.running:
            try:
                # Get user input
                cmd = input(colored(">> ", "blue")).strip()

                if cmd.lower() in ["quit", "exit", "q"]:
                    break

                if cmd:
                    # Send command to Arduino
                    control_ser.write(f"{cmd}\n".encode("utf-8"))
                    control_ser.flush()
                    print(colored(f">> {cmd}", "blue"))

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(colored(f"Input error: {e}", "red"))

    def stop_terminal(self):
        """Stop debug terminal"""
        self.running = False


def main():
    terminal = ArduinoDebugTerminal()
    try:
        terminal.start_terminal()
    except KeyboardInterrupt:
        pass
    finally:
        print(colored("\nExiting debug terminal...", "yellow"))
        terminal.stop_terminal()


if __name__ == "__main__":
    main()
