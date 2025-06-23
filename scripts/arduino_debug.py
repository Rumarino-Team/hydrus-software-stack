#!/usr/bin/env python3
"""
Arduino Debug Terminal using SOCAT virtual ports
Manual testing without stopping ROS systems
"""

import os
import time

import serial
from termcolor import colored


class ArduinoDebugTerminal:
    def __init__(self, debug_port="/dev/hydrus_debug"):
        self.debug_port = debug_port  # Send-only debug port
        self.running = False

    def start_terminal(self):
        """Start interactive debug terminal (send-only)"""
        print(colored("Arduino Debug Terminal", "cyan", attrs=["bold"]))
        print(colored("Send-only mode - Commands will be sent to Arduino", "yellow"))
        print(colored("Type 'quit' to exit", "yellow"))
        print("=" * 50)

        # Wait for debug port to be available
        while not os.path.exists(self.debug_port):
            print(f"Waiting for debug port: {self.debug_port}")
            time.sleep(1)

        try:
            # Open debug port for sending commands only
            debug_ser = serial.Serial(self.debug_port, 115200, timeout=1)

            print(
                colored("✓ Connected to Arduino debug interface (send-only)", "green")
            )
            print(colored("ℹ️  Commands will be sent directly to Arduino", "blue"))
            print(colored("ℹ️  Use monitor scripts to see Arduino responses", "blue"))

            # Start send-only command loop
            self.running = True
            self._command_loop(debug_ser)

        except Exception as e:
            print(colored(f"Failed to connect: {e}", "red"))
        finally:
            self.running = False

    def _command_loop(self, debug_ser):
        """Interactive command input loop (send-only)"""
        while self.running:
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
