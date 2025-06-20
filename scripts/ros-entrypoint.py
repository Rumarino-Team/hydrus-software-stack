#!/usr/bin/env python3
"""
Hydrus ROS Entrypoint Script - Simplified
Pure delegation to hydrus-cli for all Hydrus software operations
"""

import os
import sys
from pathlib import Path


class HydrusROSEntrypoint:
    """Simplified ROS entrypoint that delegates everything to hydrus-cli"""

    def __init__(self):
        self.script_dir = Path(__file__).parent.absolute()

        # Print environment variables for debugging
        print("=" * 60)
        print("🔧 HYDRUS ROS ENTRYPOINT")
        print("=" * 60)

        # Get all environment variables we use
        env_vars = {
            "VOLUME": os.environ.get("VOLUME", "false"),
            "TEST": os.environ.get("TEST", "false"),
            "ROSBAG_PLAYBACK": os.environ.get("ROSBAG_PLAYBACK", "false"),
            "RVIZ": os.environ.get("RVIZ", "false"),
            "ZED_OPTION": os.environ.get("ZED_OPTION", "false"),
            "DEBUG_ARDUINO": os.environ.get("DEBUG_ARDUINO", "false"),
            "NO_BUILD": os.environ.get("NO_BUILD", "false"),
            "TMUX_SESSIONS": os.environ.get("TMUX_SESSIONS", "false"),
            "ARDUINO_COMPILE": os.environ.get("ARDUINO_COMPILE", "false"),
            "VIRTUAL_ARDUINO": os.environ.get("VIRTUAL_ARDUINO", "false"),
        }

        for var_name, var_value in env_vars.items():
            print(f"📋 {var_name:<20} = {var_value}")

        print("=" * 60)
        print()

        # Parse environment variables
        self.volume = env_vars["VOLUME"].lower() == "true"
        self.test = env_vars["TEST"].lower() == "true"
        self.rosbag_playback = env_vars["ROSBAG_PLAYBACK"].lower() == "true"
        self.rviz = env_vars["RVIZ"].lower() == "true"
        self.zed_option = env_vars["ZED_OPTION"].lower() == "true"
        self.debug_arduino = env_vars["DEBUG_ARDUINO"].lower() == "true"
        self.no_build = env_vars["NO_BUILD"].lower() == "true"
        self.tmux_sessions = env_vars["TMUX_SESSIONS"].lower() == "true"
        self.arduino_compile = env_vars["ARDUINO_COMPILE"].lower() == "true"
        self.virtual_arduino = env_vars["VIRTUAL_ARDUINO"].lower() == "true"

        # Determine ROS directory based on volume usage
        if self.volume:
            print("Using the Volume directory for building the packages.")
            self.ros_dir = Path("/home/catkin_ws")
        else:
            print("Using the Copied Packages from Docker.")
            self.ros_dir = Path("/catkin_ws")

        print(f"Using ROS workspace: {self.ros_dir}")

        # Find hydrus-cli
        self.hydrus_cli_path = self._find_hydrus_cli()

    def _find_hydrus_cli(self):
        """Find hydrus-cli executable"""
        possible_locations = [
            "/usr/local/bin/hydrus-cli",
            "/usr/bin/hydrus-cli",
            self.ros_dir / "src/hydrus-software-stack/docker/hydrus-docker/hydrus-cli",
            Path(
                "/catkin_ws/src/hydrus-software-stack/docker/hydrus-docker/hydrus-cli"
            ),
        ]

        for location in possible_locations:
            if Path(location).exists():
                print(f"✅ Found hydrus-cli at: {location}")
                return Path(location)

        print("❌ hydrus-cli not found in any expected location")
        return None

    def build_hydrus_cli_command(self):
        """Build hydrus-cli command based on environment variables"""
        if not self.hydrus_cli_path:
            return None

        cmd = [str(self.hydrus_cli_path)]

        # Determine primary action
        if self.test:
            cmd.append("test")
        else:
            cmd.append("build")

        # Add flags based on environment
        if self.no_build:
            cmd.append("--no-build")
        if self.tmux_sessions:
            cmd.append("--tmux")
        if self.arduino_compile:
            cmd.append("--arduino-compile")
        if self.virtual_arduino:
            cmd.append("--virtual-arduino")
        if self.rosbag_playback:
            cmd.append("--rosbag-play")
        if self.rviz:
            cmd.append("--rviz")
        if self.debug_arduino:
            cmd.append("--serial-bridge")

        return cmd

    def main(self):
        """Main execution function - pure delegation to hydrus-cli"""
        print("\n🚀 Starting Hydrus software via hydrus-cli...")

        if not self.hydrus_cli_path:
            print("❌ hydrus-cli not found! Cannot start Hydrus software.")
            print("🔧 Please ensure hydrus-cli is installed in the container.")
            print("📍 Expected locations:")
            print("   - /usr/local/bin/hydrus-cli")
            print("   - /usr/bin/hydrus-cli")
            print(
                "   - {workspace}/src/hydrus-software-stack/docker/hydrus-docker/hydrus-cli"
            )
            sys.exit(1)

        # Build command
        cmd = self.build_hydrus_cli_command()

        if cmd:
            print(f"💻 Executing: {' '.join(cmd)}")
            print("=" * 60)

            # Make hydrus-cli executable and replace current process
            try:
                os.chmod(str(self.hydrus_cli_path), 0o755)
                os.execv(str(self.hydrus_cli_path), cmd)
            except Exception as e:
                print(f"❌ Failed to execute hydrus-cli: {e}")
                sys.exit(1)
        else:
            print("❌ Failed to build hydrus-cli command")
            sys.exit(1)


if __name__ == "__main__":
    entrypoint = HydrusROSEntrypoint()
    entrypoint.main()
