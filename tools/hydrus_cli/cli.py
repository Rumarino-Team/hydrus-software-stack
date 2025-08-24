#!/usr/bin/env python3
import typer

from arduino import arduino_command
from autonomy import autonomy_app
from test import test_app
from tmux import tmux_command
from todo import todo_app

app = typer.Typer()

# Add subcommands
app.add_typer(
    arduino_command, name="arduino", help="Arduino device management commands"
)
app.add_typer(
    autonomy_app, name="autonomy", help="Autonomy system and CV pipeline management"
)
# Note: ROS app is commented out until implemented
# app.add_typer(ros_app, name="ros", help="ROS workspace and utilities management")
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
