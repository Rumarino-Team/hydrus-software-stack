#!/usr/bin/env python3
import typer

from .commands.arduino import arduino_command
from .commands.ros import ros_app
from .commands.test import test_app
from .commands.tmux import tmux_command
from .commands.todo import todo_app

app = typer.Typer()

# Add subcommands
app.add_typer(
    arduino_command, name="arduino", help="Arduino device management commands"
)
app.add_typer(ros_app, name="ros", help="ROS workspace and utilities management")
app.add_typer(test_app, name="test", help="Test suite management and execution")
app.add_typer(tmux_command, name="tmux", help="Tmux session management commands")
app.add_typer(todo_app, name="todo", help="TODO and FIXME tracker for code analysis")


@app.command()
def hello():
    """Say hello from Hydrus CLI."""
    typer.echo("Hello from Hydrus Software Stack!")
    typer.echo("Use --help to see available commands.")


if __name__ == "__main__":
    app()
