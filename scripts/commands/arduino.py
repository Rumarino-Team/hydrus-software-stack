import typer

from .utils import Command

arduino_command = typer.Typer()


class ArduinoCommand(Command):
    """
    Command to interact with Arduino devices.
    """

    def __init__(self):
        super().__init__(name="arduino", description="Interact with Arduino devices")
