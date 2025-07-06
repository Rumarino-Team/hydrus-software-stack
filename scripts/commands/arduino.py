import time

import typer

from devices.Arduino.arduino import Arduino
from devices.Arduino.virtual_arduino import check_and_create_virtual_arduino

arduino_command = typer.Typer()


@arduino_command.command()
def multiplexer(device: str = "/dev/ttyACM0"):
    """Start the Arduino multiplexer."""
    arduino = Arduino(device)
    if arduino.start_multiplexer():
        typer.echo("Arduino multiplexer started successfully!")
    else:
        typer.echo("Failed to start Arduino multiplexer.")


@arduino_command.command()
def debug():
    """Start the Arduino debug terminal (send-only)."""
    Arduino.arduino_debug_terminal()


@arduino_command.command()
def monitor():
    """Start the Arduino monitor (real-time monitoring)."""
    Arduino.arduino_monitor()


@arduino_command.command()
def virtual(device: str = "/dev/ttyACM0"):
    """Start a virtual Arduino device if no real Arduino is present."""
    va = check_and_create_virtual_arduino(device)
    if va:
        typer.echo(f"Virtual Arduino started at {device}. Press Ctrl+C to stop.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            va.stop()
    else:
        typer.echo("Using real Arduino device.")
