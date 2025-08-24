#!/usr/bin/env python3
"""
Hydrus Tmux Session Manager - Typer Command Interface
Replaces start_tmux_sessions.sh with improved modularity and error handling
"""

import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional

import typer

from scripts.commands.utils import get_building_path

tmux_command = typer.Typer()


class HydrusTmuxManager:
    def __init__(self):
        self.volume = True  # os.environ.get("VOLUME", "false").lower() == "true"
        self.catkin_ws = get_building_path(self.volume)
        # Window configuration
        self.window_config = self._get_window_configuration()

    def _get_window_configuration(self) -> dict:
        """Define tmux window configuration"""
        source_cmd = f"source {self.catkin_ws}/devel/setup.bash"
        arduino_port = self._get_arduino_port()

        return {
            "Controls": {
                "window_index": 0,
                "layout": "main-horizontal",
                "panes": [
                    {
                        "name": "Serial ROS Bridge",
                        "command": f"echo 'Starting Serial ROS Bridge (using {arduino_port})'; {source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/scripts/controller/serial_ros_bridge.py _port:={arduino_port} _baud_rate:=115200",
                        "split": None,  # First pane, no split
                    },
                    {
                        "name": "Submarine Teleop + Visualizer",
                        "command": f"echo 'Starting Submarine Teleoperation with Live Visualization'; {source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/scripts/controller/thruster_visualizer.py",
                        "split": "vertical",  # Split vertically from previous pane
                    },
                    {
                        "name": "Controller Node",
                        "command": f"echo 'Starting Controller Node'; {source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/src/controllers.py",
                        "split": "horizontal",  # Split horizontally from previous pane
                    },
                ],
            },
            "Arduino": {
                "window_index": 1,
                "layout": "even-horizontal",
                "panes": [
                    {
                        "name": "Arduino Multiplexer",
                        "command": f"echo 'Starting Arduino Multiplexer'; python3 {self.catkin_ws}/src/hydrus-software-stack/devices/Arduino/arduino.py multiplexer {arduino_port}",
                        "split": None,
                    },
                    {
                        "name": "Arduino Monitor",
                        "command": f"sleep 3; echo 'Starting Arduino Monitor'; python3 {self.catkin_ws}/src/hydrus-software-stack/devices/Arduino/arduino.py monitor",
                        "split": "horizontal",
                    },
                    {
                        "name": "Debug Terminal",
                        "command": f"sleep 5; echo 'Starting Arduino Debug Terminal'; python3 {self.catkin_ws}/src/hydrus-software-stack/devices/Arduino/arduino.py debug",
                        "split": "vertical",
                    },
                ],
            },
            "Computer Vision": {
                "window_index": 2,
                "layout": "main-vertical",
                "panes": [
                    {
                        "name": "Color Filter Controller",
                        "command": f"echo 'Starting Color Filter Controller'; {source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/scripts/cv/color_filter_controller.py ui",
                        "split": None,
                    },
                    {
                        "name": "CV Publishers",
                        "command": f"echo 'Starting Computer Vision Publishers'; {source_cmd} && roslaunch autonomy cv_publishers.launch",
                        "split": "horizontal",
                    },
                    {
                        "name": "Web Detection Viewer",
                        "command": f"echo 'Starting Web Detection Viewer'; {source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/scripts/web/detection_viewer.py",
                        "split": "vertical",
                    },
                    {
                        "name": "API Server",
                        "command": f"echo 'Starting API Server'; {source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/src/api_server.py",
                        "split": "horizontal",
                    },
                ],
            },
            "Mission Planner": {
                "window_index": 3,
                "layout": "tiled",
                "panes": [
                    {
                        "name": "Mission Manager",
                        "command": f"echo 'Starting Mission Manager'; {source_cmd} && roslaunch autonomy mission_planner.launch",
                        "split": None,
                    },
                    {
                        "name": "Mission Controller",
                        "command": f"echo 'Starting Mission Controller'; {source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/scripts/mission/mission_controller.py",
                        "split": "horizontal",
                    },
                    {
                        "name": "Controller Monitor",
                        "command": f"echo 'Starting Controller Monitor'; {source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/scripts/controller/controller_monitor.py",
                        "split": "vertical",
                    },
                ],
            },
        }

    def _run_command(
        self, cmd: List[str], check: bool = True, capture_output: bool = False
    ) -> subprocess.CompletedProcess:
        """Run a command with proper error handling"""
        try:
            return subprocess.run(
                cmd, check=check, capture_output=capture_output, text=True
            )
        except subprocess.CalledProcessError as e:
            if check:
                typer.echo(f"Command failed: {' '.join(cmd)}")
                typer.echo(f"Exit code: {e.returncode}")
            raise

    def _check_tmux_installed(self):
        """Check if tmux is installed and install if needed"""
        try:
            self._run_command(["tmux", "-V"], capture_output=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            typer.echo("tmux could not be found. Installing tmux...")
            self._run_command(["apt-get", "update"])
            self._run_command(["apt-get", "install", "-y", "tmux"])

    def _setup_virtual_arduino(self):
        """Setup virtual Arduino if needed"""
        typer.echo("Checking for Arduino device...")
        virtual_arduino_script = (
            self.catkin_ws / "src/hydrus-software-stack/scripts/virtual_arduino.py"
        )

        try:
            subprocess.run(
                ["python3", str(virtual_arduino_script), "/dev/ttyACM0"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(3)  # Give time for virtual Arduino to start
        except Exception as e:
            typer.echo(f"Failed to start virtual Arduino: {e}")

    def _setup_serial_monitor(self):
        """Setup Arduino serial monitoring if not already running"""
        if not Path("/tmp/hydrus_serial/catpid.txt").exists():
            typer.echo("Setting up Arduino serial monitoring...")
            setup_script = (
                self.catkin_ws
                / "src/hydrus-software-stack/scripts/setup_serial_monitor.py"
            )
            if setup_script.exists():
                try:
                    subprocess.run(
                        [sys.executable, str(setup_script), "/dev/ttyACM0"], check=False
                    )
                except Exception as e:
                    typer.echo(f"Failed to setup serial monitor: {e}")
            else:
                # Fallback to bash script
                setup_script = (
                    self.catkin_ws
                    / "src/hydrus-software-stack/scripts/setup_serial_monitor.sh"
                )
                if setup_script.exists():
                    subprocess.run(
                        ["bash", str(setup_script), "/dev/ttyACM0"], check=False
                    )
            time.sleep(2)

    def _kill_existing_session(self):
        """Kill existing tmux session if it exists"""
        try:
            self._run_command(
                ["tmux", "kill-session", "-t", "hydrus"],
                check=False,
                capture_output=True,
            )
        except Exception:
            pass

    def _tmux_command(self, cmd: List[str]):
        """Execute a tmux command with error handling"""
        try:
            self._run_command(["tmux"] + cmd, check=False)
        except Exception as e:
            typer.echo(f"Tmux command failed: {' '.join(cmd)}: {e}")

    def _create_window_from_config(self, window_name: str, config: dict):
        """Create a tmux window based on configuration"""
        typer.echo(f"Creating {window_name} window...")

        window_index = config["window_index"]
        layout = config.get("layout", "even-horizontal")
        panes = config["panes"]

        # Create the window (first window uses new-session, others use new-window)
        if window_index == 0:
            self._tmux_command(["new-session", "-d", "-s", "hydrus", "-n", window_name])
        else:
            self._tmux_command(
                ["new-window", "-t", f"hydrus:{window_index}", "-n", window_name]
            )

        # Create panes
        for i, pane in enumerate(panes):
            command = pane["command"]
            split_type = pane.get("split")

            if i == 0:
                # First pane - just send the command
                target = f"hydrus:{window_index}"
            else:
                # Subsequent panes - create splits
                if split_type == "vertical":
                    self._tmux_command(
                        ["split-window", "-v", "-t", f"hydrus:{window_index}"]
                    )
                elif split_type == "horizontal":
                    self._tmux_command(
                        ["split-window", "-h", "-t", f"hydrus:{window_index}"]
                    )

                target = f"hydrus:{window_index}.{i}"

            # Send command to the pane
            self._tmux_command(["send-keys", "-t", target, command, "C-m"])

        # Apply layout
        if layout:
            self._tmux_command(
                ["select-layout", "-t", f"hydrus:{window_index}", layout]
            )

    def _create_all_windows(self, window_filter: Optional[List[str]] = None):
        """Create all tmux windows from configuration"""
        typer.echo("Creating tmux windows from configuration...")

        # Sort windows by index to ensure proper creation order
        sorted_windows = sorted(
            self.window_config.items(), key=lambda x: x[1]["window_index"]
        )

        # Filter windows if specified
        if window_filter:
            sorted_windows = [
                (name, config)
                for name, config in sorted_windows
                if name in window_filter
            ]

        for window_name, config in sorted_windows:
            try:
                self._create_window_from_config(window_name, config)
            except Exception as e:
                typer.echo(f"Failed to create {window_name} window: {e}")
                continue

    def _finalize_session(self):
        """Finalize tmux session setup"""
        # Return to the main control window and select teleop pane
        self._tmux_command(["select-window", "-t", "hydrus:0"])
        self._tmux_command(["select-pane", "-t", "hydrus:0.1"])

    def _get_command_with_fallback(
        self, primary_script: str, fallback_script: str, description: str
    ) -> str:
        """Get command with fallback if primary script doesn't exist"""
        primary_path = self.catkin_ws / "src/hydrus-software-stack" / primary_script
        fallback_path = self.catkin_ws / "src/hydrus-software-stack" / fallback_script

        if primary_path.exists():
            return f"echo 'Starting {description}'; python3 {primary_path}"
        elif fallback_path.exists():
            return f"echo 'Starting {description} (fallback)'; python3 {fallback_path}"
        else:
            return f"echo 'Error: {description} script not found'; echo 'Available commands:'; echo '  ls {self.catkin_ws}/src/hydrus-software-stack/scripts/'; ls {self.catkin_ws}/src/hydrus-software-stack/scripts/ | grep -E '(arduino|serial|monitor)' || echo '  No matching scripts found'"

    def _get_arduino_port(self) -> str:
        """Get the appropriate Arduino port (virtual if available, physical otherwise)"""
        if os.path.exists("/dev/hydrus_control"):
            return "/dev/hydrus_control"
        else:
            return "/dev/ttyACM0"

    def create_session(self, window_filter: Optional[List[str]] = None):
        """Main execution function"""
        try:
            # Check and install tmux if needed
            self._check_tmux_installed()

            # Setup serial monitor for Arduino (ignore errors)
            self._setup_virtual_arduino()
            self._setup_serial_monitor()

            # Kill existing tmux session
            self._kill_existing_session()

            typer.echo("Creating tmux session 'hydrus'...")

            # Create all windows from configuration
            self._create_all_windows(window_filter)

            # Finalize session
            self._finalize_session()

            typer.echo(
                f"Tmux session 'hydrus' created with catkin workspace at {self.catkin_ws}."
            )
            typer.echo("You can attach to it using 'tmux attach -t hydrus'.")

        except Exception as e:
            typer.echo(f"Failed to create tmux session: {e}")
            raise typer.Exit(1)


@tmux_command.command("start")
def start_session(
    windows: Optional[List[str]] = typer.Option(
        None, "--windows", "-w", help="Only create specified windows"
    ),
    config_file: Optional[str] = typer.Option(
        None, "--config", "-c", help="Load window configuration from JSON file"
    ),
):
    """Start and manage tmux monitoring sessions."""
    manager = HydrusTmuxManager()

    # Load custom configuration if specified
    if config_file:
        config_path = Path(config_file)
        if config_path.exists():
            try:
                with open(config_path, "r") as f:
                    loaded_config = json.load(f)
                manager.window_config.update(loaded_config)
                typer.echo(f"Loaded configuration from {config_path}")
            except Exception as e:
                typer.echo(f"Failed to load configuration: {e}")
                raise typer.Exit(1)
        else:
            typer.echo(f"Configuration file not found: {config_path}")
            raise typer.Exit(1)

    manager.create_session(windows)


@tmux_command.command("list-windows")
def list_windows():
    """List all configured tmux windows."""
    manager = HydrusTmuxManager()
    typer.echo("Configured windows:")
    for window in manager.window_config.keys():
        typer.echo(f"  - {window}")


@tmux_command.command("save-config")
def save_config(
    output_file: str = typer.Argument(..., help="Output file path for configuration")
):
    """Save current window configuration to JSON file."""
    manager = HydrusTmuxManager()
    try:
        with open(output_file, "w") as f:
            json.dump(manager.window_config, f, indent=2)
        typer.echo(f"Configuration saved to {output_file}")
    except Exception as e:
        typer.echo(f"Failed to save configuration: {e}")
        raise typer.Exit(1)


@tmux_command.command("validate")
def validate_config():
    """Validate current window configuration."""
    manager = HydrusTmuxManager()
    typer.echo("Validating configuration...")

    all_valid = True
    for window_name, config in manager.window_config.items():
        # Basic validation
        required_keys = ["window_index", "panes"]
        valid = all(key in config for key in required_keys)

        if valid and isinstance(config["panes"], list) and len(config["panes"]) > 0:
            typer.echo(f"✓ {window_name}: Valid")
        else:
            typer.echo(f"✗ {window_name}: Invalid")
            all_valid = False

    if all_valid:
        typer.echo("All configurations are valid!")
    else:
        typer.echo("Some configurations are invalid!")
        raise typer.Exit(1)


@tmux_command.command("kill")
def kill_session():
    """Kill the hydrus tmux session."""
    try:
        subprocess.run(
            ["tmux", "kill-session", "-t", "hydrus"], check=True, capture_output=True
        )
        typer.echo("Hydrus tmux session terminated.")
    except subprocess.CalledProcessError:
        typer.echo("No hydrus tmux session found.")
    except Exception as e:
        typer.echo(f"Failed to kill session: {e}")
        raise typer.Exit(1)


@tmux_command.command("attach")
def attach_session():
    """Attach to the hydrus tmux session."""
    try:
        # Check if session exists
        result = subprocess.run(
            ["tmux", "has-session", "-t", "hydrus"], capture_output=True
        )
        if result.returncode == 0:
            os.system("tmux attach -t hydrus")
        else:
            typer.echo("No hydrus tmux session found. Use 'tmux start' to create one.")
    except Exception as e:
        typer.echo(f"Failed to attach to session: {e}")
        raise typer.Exit(1)


if __name__ == "__main__":
    tmux_command()
