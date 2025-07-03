#!/usr/bin/env python3
"""
Hydrus ROS Entrypoint Script - Simplified
Pure delegation to hydrus-cli for all Hydrus software operations
"""

import os
import sys
from pathlib import Path

from scripts.scripts_utils import get_building_path


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

        self.ros_dir = get_building_path(self.volume)

        print(f"Using ROS workspace: {self.ros_dir}")

        # Always use hydrus-cli from the repo inside the ROS workspace
        self.hydrus_cli_path = self.ros_dir / "src/hydrus-software-stack/hydrus-cli.py"

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

        # Build command
        cmd = self.build_hydrus_cli_command()

        if cmd:
            print(f"💻 Executing: {' '.join(cmd)}")
            print("=" * 60)

            # Make hydrus-cli executable
            try:
                if self.test:
                    # In test mode, execute and exit when done
                    os.execv(str(self.hydrus_cli_path), cmd)
                else:
                    # In non-test mode, run hydrus-cli and then keep container alive
                    import subprocess
                    import time

                    # Start hydrus-cli in background
                    print("🔄 Running in production mode - container will stay alive")
                    process = subprocess.Popen(cmd)

                    try:
                        # Wait for the process and keep container alive
                        while True:
                            # Check if process is still running
                            if process.poll() is not None:
                                print(
                                    f"⚠️  hydrus-cli process exited with code {process.returncode}"
                                )
                                print("🔄 Keeping container alive...")

                            # Sleep to prevent high CPU usage
                            time.sleep(10)

                    except KeyboardInterrupt:
                        print("\n🛑 Received interrupt signal, shutting down...")
                        process.terminate()
                        process.wait()
                        sys.exit(0)

            except Exception as e:
                print(f"❌ Failed to execute hydrus-cli: {e}")
                sys.exit(1)
        else:
            print("❌ Failed to build hydrus-cli command")
            sys.exit(1)


if __name__ == "__main__":
    entrypoint = HydrusROSEntrypoint()
    entrypoint.main()
