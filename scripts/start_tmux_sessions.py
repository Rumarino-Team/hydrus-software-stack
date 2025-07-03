#!/usr/bin/env python3
"""
Hydrus Tmux Session Manager
Replaces start_tmux_sessions.sh with improved modularity and error handling
"""

import os
import subprocess
import sys
import time
from pathlib import Path
from typing import List

from scripts.scripts_utils import get_building_path


class HydrusTmuxManager:
    def __init__(self):
        self.volume = os.environ.get("VOLUME", "false").lower() == "true"
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
                        "command": self._get_command_with_fallback(
                            "scripts/simple_arduino_multiplexer.py",
                            "scripts/socat_arduino_multiplexer.py",
                            "Arduino Multiplexer",
                        ),
                        "split": None,
                    },
                    {
                        "name": "Arduino Monitor",
                        "command": "sleep 3; "
                        + self._get_command_with_fallback(
                            "scripts/monitor_arduino_logs.py",
                            "scripts/setup_serial_monitor.py",
                            "Arduino Monitor",
                        ),
                        "split": "horizontal",
                    },
                    {
                        "name": "Debug Terminal",
                        "command": "sleep 5; "
                        + self._get_command_with_fallback(
                            "scripts/arduino_debug.py",
                            "scripts/arduino_debug_terminal.py",
                            "Arduino Debug Terminal",
                        ),
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
                print(f"Command failed: {' '.join(cmd)}")
                print(f"Exit code: {e.returncode}")
            raise

    def _check_tmux_installed(self):
        """Check if tmux is installed and install if needed"""
        try:
            self._run_command(["tmux", "-V"], capture_output=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("tmux could not be found. Installing tmux...")
            self._run_command(["apt-get", "update"])
            self._run_command(["apt-get", "install", "-y", "tmux"])

    def _setup_virtual_arduino(self):
        """Setup virtual Arduino if needed"""
        print("Checking for Arduino device...")
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
            print(f"Failed to start virtual Arduino: {e}")

    def _setup_serial_monitor(self):
        """Setup Arduino serial monitoring if not already running"""
        if not Path("/tmp/hydrus_serial/catpid.txt").exists():
            print("Setting up Arduino serial monitoring...")
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
                    print(f"Failed to setup serial monitor: {e}")
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
            print(f"Tmux command failed: {' '.join(cmd)}: {e}")

    def _create_window_from_config(self, window_name: str, config: dict):
        """Create a tmux window based on configuration"""
        print(f"Creating {window_name} window...")

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

    def _create_all_windows(self):
        """Create all tmux windows from configuration"""
        print("Creating tmux windows from configuration...")

        # Sort windows by index to ensure proper creation order
        sorted_windows = sorted(
            self.window_config.items(), key=lambda x: x[1]["window_index"]
        )

        for window_name, config in sorted_windows:
            try:
                self._create_window_from_config(window_name, config)
            except Exception as e:
                print(f"Failed to create {window_name} window: {e}")
                continue

    def _finalize_session(self):
        """Finalize tmux session setup"""
        # Return to the main control window and select teleop pane
        self._tmux_command(["select-window", "-t", "hydrus:0"])
        self._tmux_command(["select-pane", "-t", "hydrus:0.1"])

    def add_window_config(self, window_name: str, config: dict):
        """Add a new window configuration"""
        self.window_config[window_name] = config

    def remove_window_config(self, window_name: str):
        """Remove a window configuration"""
        if window_name in self.window_config:
            del self.window_config[window_name]

    def get_window_config(self, window_name: str) -> dict:
        """Get configuration for a specific window"""
        return self.window_config.get(window_name, {})

    def list_windows(self) -> List[str]:
        """List all configured window names"""
        return list(self.window_config.keys())

    def validate_window_config(self, config: dict) -> bool:
        """Validate window configuration structure"""
        required_keys = ["window_index", "panes"]

        # Check required keys
        for key in required_keys:
            if key not in config:
                print(f"Missing required key: {key}")
                return False

        # Check panes structure
        panes = config["panes"]
        if not isinstance(panes, list) or len(panes) == 0:
            print("Panes must be a non-empty list")
            return False

        for i, pane in enumerate(panes):
            if not isinstance(pane, dict):
                print(f"Pane {i} must be a dictionary")
                return False

            if "command" not in pane:
                print(f"Pane {i} missing required 'command' key")
                return False

            # First pane should not have split
            if i == 0 and pane.get("split") is not None:
                print("First pane should not have 'split' defined")
                return False

            # Other panes should have split
            if i > 0 and "split" not in pane:
                print(f"Pane {i} missing required 'split' key")
                return False

        return True

    def load_config_from_file(self, config_file: Path):
        """Load window configuration from JSON file"""
        import json

        try:
            with open(config_file, "r") as f:
                loaded_config = json.load(f)

            # Validate each window config
            for window_name, config in loaded_config.items():
                if self.validate_window_config(config):
                    self.window_config[window_name] = config
                else:
                    print(
                        f"Invalid configuration for window '{window_name}', skipping..."
                    )

        except Exception as e:
            print(f"Failed to load configuration from {config_file}: {e}")

    def save_config_to_file(self, config_file: Path):
        """Save current window configuration to JSON file"""
        import json

        try:
            with open(config_file, "w") as f:
                json.dump(self.window_config, f, indent=2)
            print(f"Configuration saved to {config_file}")
        except Exception as e:
            print(f"Failed to save configuration to {config_file}: {e}")

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

    def main(self):
        """Main execution function"""
        try:
            # Check and install tmux if needed
            self._check_tmux_installed()

            # Setup serial monitor for Arduino (ignore errors)
            self._setup_virtual_arduino()
            self._setup_serial_monitor()

            # Kill existing tmux session
            self._kill_existing_session()

            print("Creating tmux session 'hydrus'...")

            # Create all windows from configuration
            self._create_all_windows()

            # Finalize session
            self._finalize_session()

            print(
                f"Tmux session 'hydrus' created with catkin workspace at {self.catkin_ws}."
            )
            print("You can attach to it using 'tmux attach -t hydrus'.")

        except Exception as e:
            print(f"Failed to create tmux session: {e}")
            sys.exit(1)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Hydrus Tmux Session Manager")
    parser.add_argument(
        "--config", type=str, help="Load window configuration from JSON file"
    )
    parser.add_argument(
        "--save-config", type=str, help="Save current configuration to JSON file"
    )
    parser.add_argument(
        "--list-windows", action="store_true", help="List all configured windows"
    )
    parser.add_argument(
        "--validate", action="store_true", help="Validate current configuration"
    )
    parser.add_argument(
        "--windows", type=str, nargs="*", help="Only create specified windows"
    )

    args = parser.parse_args()

    manager = HydrusTmuxManager()

    # Load custom configuration if specified
    if args.config:
        config_path = Path(args.config)
        if config_path.exists():
            manager.load_config_from_file(config_path)
        else:
            print(f"Configuration file not found: {config_path}")
            sys.exit(1)

    # Handle utility commands
    if args.list_windows:
        print("Configured windows:")
        for window in manager.list_windows():
            print(f"  - {window}")
        sys.exit(0)

    if args.validate:
        print("Validating configuration...")
        all_valid = True
        for window_name, config in manager.window_config.items():
            if manager.validate_window_config(config):
                print(f"✓ {window_name}: Valid")
            else:
                print(f"✗ {window_name}: Invalid")
                all_valid = False

        if all_valid:
            print("All configurations are valid!")
        else:
            print("Some configurations are invalid!")
            sys.exit(1)
        sys.exit(0)

    if args.save_config:
        manager.save_config_to_file(Path(args.save_config))
        sys.exit(0)

    # Filter windows if specified
    if args.windows:
        filtered_config = {}
        for window_name in args.windows:
            if window_name in manager.window_config:
                filtered_config[window_name] = manager.window_config[window_name]
            else:
                print(f"Warning: Window '{window_name}' not found in configuration")

        if filtered_config:
            manager.window_config = filtered_config
        else:
            print("No valid windows specified!")
            sys.exit(1)

    # Run the main session creation
    manager.main()
