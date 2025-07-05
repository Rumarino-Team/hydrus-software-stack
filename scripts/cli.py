#!/usr/bin/env python3
import typer

from .commands.arduino import ArduinoCommand
from .commands.ros import HydrusRosManager
from .commands.tmux import HydrusTmuxManager

app = typer.Typer()


class HydrusClI:
    def __init__(self):
        self.ros_manager = HydrusRosManager()
        self.tmux_manager = HydrusTmuxManager()


if __name__ == "__main__":
    app()
