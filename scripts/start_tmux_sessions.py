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
from typing import List, Optional


class HydrusTmuxManager:
    def __init__(self):
        self.volume = os.environ.get("VOLUME", "false").lower() == "true"

        # Determine ROS directory based on volume usage
        if self.volume:
            print("Using Volume directory for tmux scripts: /home/catkin_ws")
            self.catkin_ws = Path("/home/catkin_ws")
        else:
            print("Using Docker container directory: /catkin_ws")
            self.catkin_ws = Path("/catkin_ws")

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
            process = subprocess.Popen(
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
        except:
            pass

    def _tmux_command(self, cmd: List[str]):
        """Execute a tmux command with error handling"""
        try:
            self._run_command(["tmux"] + cmd, check=False)
        except Exception as e:
            print(f"Tmux command failed: {' '.join(cmd)}: {e}")

    def _create_controls_window(self):
        """Create the Controls window with controller components"""
        print("Creating Controls window...")

        # Create new session with Controls window
        self._tmux_command(["new-session", "-d", "-s", "hydrus", "-n", "Controls"])

        # First pane: Serial ROS Bridge
        source_cmd = f"source {self.catkin_ws}/devel/setup.bash"
        bridge_cmd = f"{source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/scripts/controller/serial_ros_bridge.py _port:=/dev/ttyACM0 _baud_rate:=115200"

        self._tmux_command(
            [
                "send-keys",
                "-t",
                "hydrus",
                f"echo 'Starting Serial ROS Bridge'; {bridge_cmd}",
                "C-m",
            ]
        )

        # Second pane: Controllers
        self._tmux_command(["split-window", "-v", "-t", "hydrus"])
        controller_cmd = f"{source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/src/controllers.py"

        self._tmux_command(
            [
                "send-keys",
                "-t",
                "hydrus:0.1",
                f"echo 'Starting Controller Node'; {controller_cmd}",
                "C-m",
            ]
        )

        # Third pane: Thruster Visualizer
        self._tmux_command(["split-window", "-h", "-t", "hydrus:0.1"])
        visualizer_cmd = f"{source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/scripts/controller/thruster_visualizer.py"

        self._tmux_command(
            [
                "send-keys",
                "-t",
                "hydrus:0.2",
                f"echo 'Starting Thruster Visualizer'; {visualizer_cmd}",
                "C-m",
            ]
        )

    def _create_arduino_window(self):
        """Create the Arduino monitoring window"""
        print("Creating Arduino window...")

        # Create new window for Arduino monitoring
        self._tmux_command(["new-window", "-t", "hydrus:1", "-n", "Arduino"])

        # First pane: Setup serial monitor
        setup_script_py = (
            self.catkin_ws / "src/hydrus-software-stack/scripts/setup_serial_monitor.py"
        )

        if setup_script_py.exists():
            setup_cmd = f"python3 {setup_script_py}"
        else:
            setup_cmd = "echo 'No setup script found'"

        self._tmux_command(
            [
                "send-keys",
                "-t",
                "hydrus:1.0",
                f"echo 'Setting up Arduino monitoring'; {setup_cmd}",
                "C-m",
            ]
        )

        # Second pane: Monitor Arduino logs
        self._tmux_command(["split-window", "-h", "-t", "hydrus:1"])
        monitor_script_py = (
            self.catkin_ws / "src/hydrus-software-stack/scripts/monitor_arduino_logs.py"
        )

        if monitor_script_py.exists():
            monitor_cmd = f"python3 {monitor_script_py}"
        else:
            monitor_cmd = "echo 'No monitor script found'"

        self._tmux_command(
            [
                "send-keys",
                "-t",
                "hydrus:1.1",
                f"echo 'Starting Arduino log monitor'; sleep 2; {monitor_cmd}",
                "C-m",
            ]
        )

        # Set layout for Arduino window
        self._tmux_command(["select-layout", "-t", "hydrus:1", "even-horizontal"])

    def _create_cv_window(self):
        """Create the Computer Vision window"""
        print("Creating Computer Vision window...")

        # Create new window for Computer Vision
        self._tmux_command(["new-window", "-t", "hydrus:2", "-n", "Computer Vision"])

        source_cmd = f"source {self.catkin_ws}/devel/setup.bash"

        # First pane: Color Filter Controller (taking half the screen)
        color_filter_cmd = f"{source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/scripts/cv/color_filter_controller.py ui"

        self._tmux_command(
            [
                "send-keys",
                "-t",
                "hydrus:2.0",
                f"echo 'Starting Color Filter Controller'; {color_filter_cmd}",
                "C-m",
            ]
        )

        # Second pane: CV Publishers (right side, top half)
        self._tmux_command(["split-window", "-h", "-t", "hydrus:2.0"])
        cv_publishers_cmd = f"{source_cmd} && roslaunch autonomy cv_publishers.launch"

        self._tmux_command(
            [
                "send-keys",
                "-t",
                "hydrus:2.1",
                f"echo 'Starting Computer Vision Publishers'; {cv_publishers_cmd}",
                "C-m",
            ]
        )

        # Third pane: Web Detection Viewer (right side, bottom left)
        self._tmux_command(["split-window", "-v", "-t", "hydrus:2.1"])
        detection_viewer_cmd = f"{source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/scripts/web/detection_viewer.py"

        self._tmux_command(
            [
                "send-keys",
                "-t",
                "hydrus:2.2",
                f"echo 'Starting Web Detection Viewer'; {detection_viewer_cmd}",
                "C-m",
            ]
        )

        # Fourth pane: API Server (right side, bottom right)
        self._tmux_command(["split-window", "-h", "-t", "hydrus:2.2"])
        api_cmd = f"{source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/src/api_server.py"

        self._tmux_command(
            [
                "send-keys",
                "-t",
                "hydrus:2.3",
                f"echo 'Starting API Server'; {api_cmd}",
                "C-m",
            ]
        )

        # Resize panes for desired layout
        # Get terminal columns for resizing
        try:
            result = subprocess.run(["tput", "cols"], capture_output=True, text=True)
            cols = int(result.stdout.strip()) if result.returncode == 0 else 80
            half_cols = cols // 2
            quarter_cols = cols // 4

            # Resize controller pane to take 50% of width
            self._tmux_command(
                ["resize-pane", "-t", "hydrus:2.0", "-x", str(half_cols)]
            )

            # Resize bottom panes to be equal width
            self._tmux_command(["select-pane", "-t", "hydrus:2.2"])
            self._tmux_command(["resize-pane", "-x", str(quarter_cols)])
        except:
            pass  # Continue without resizing if it fails

    def _create_mission_planner_window(self):
        """Create the Mission Planner window"""
        print("Creating Mission Planner window...")

        # Create new window for Mission Planning
        self._tmux_command(["new-window", "-t", "hydrus:3", "-n", "Mission Planner"])

        source_cmd = f"source {self.catkin_ws}/devel/setup.bash"

        # First pane: Mission Manager
        mission_manager_cmd = (
            f"{source_cmd} && roslaunch autonomy mission_planner.launch"
        )

        self._tmux_command(
            [
                "send-keys",
                "-t",
                "hydrus:3.0",
                f"echo 'Starting Mission Manager'; {mission_manager_cmd}",
                "C-m",
            ]
        )

        # Second pane: Mission Controller
        self._tmux_command(["split-window", "-h", "-t", "hydrus:3.0"])
        mission_controller_cmd = f"{source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/scripts/mission/mission_controller.py"

        self._tmux_command(
            [
                "send-keys",
                "-t",
                "hydrus:3.1",
                f"echo 'Starting Mission Controller'; {mission_controller_cmd}",
                "C-m",
            ]
        )

        # Third pane: Controller Monitor
        self._tmux_command(["split-window", "-v", "-t", "hydrus:3.1"])
        controller_monitor_cmd = f"{source_cmd} && python3 {self.catkin_ws}/src/hydrus-software-stack/autonomy/scripts/controller/controller_monitor.py"

        self._tmux_command(
            [
                "send-keys",
                "-t",
                "hydrus:3.2",
                f"echo 'Starting Controller Monitor'; {controller_monitor_cmd}",
                "C-m",
            ]
        )

        # Set tiled layout for Mission Planning window
        self._tmux_command(["select-layout", "-t", "hydrus:3", "tiled"])

    def _finalize_session(self):
        """Finalize tmux session setup"""
        # Return to the main control window and select teleop pane
        self._tmux_command(["select-window", "-t", "hydrus:0"])
        self._tmux_command(["select-pane", "-t", "hydrus:0.1"])

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

            # Create all windows
            self._create_controls_window()
            self._create_arduino_window()
            self._create_cv_window()
            self._create_mission_planner_window()

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
    manager = HydrusTmuxManager()
    manager.main()
