#!/usr/bin/env python3
"""
Hydrus Camera Entrypoint Script
Replaces camera-entrypoint.sh with improved error handling
"""

import os
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional


class CameraEntrypoint:
    def __init__(self):
        self.zed_option = os.environ.get("ZED_OPTION", "false").lower() == "true"
        self.workspace_dir = Path("/catkin_ws")

    def _run_command(
        self,
        cmd: List[str],
        check: bool = True,
        capture_output: bool = False,
        env: Optional[Dict] = None,
        cwd: Optional[Path] = None,
    ) -> subprocess.CompletedProcess:
        """Run a command with proper error handling"""
        if env is None:
            env = os.environ.copy()

        try:
            return subprocess.run(
                cmd,
                check=check,
                capture_output=capture_output,
                env=env,
                cwd=cwd,
                text=True,
            )
        except subprocess.CalledProcessError as e:
            if check:
                print(f"Command failed: {' '.join(cmd)}")
                print(f"Exit code: {e.returncode}")
                if capture_output and e.stderr:
                    print(f"Error: {e.stderr}")
            raise

    def _setup_environment(self) -> Dict[str, str]:
        """Setup ROS environment"""
        env = os.environ.copy()

        # Source devel/setup.bash equivalent
        setup_file = self.workspace_dir / "devel/setup.bash"
        if setup_file.exists():
            # Add workspace paths to environment
            env[
                "ROS_PACKAGE_PATH"
            ] = f"{self.workspace_dir}/src:{env.get('ROS_PACKAGE_PATH', '')}"
            env[
                "CMAKE_PREFIX_PATH"
            ] = f"{self.workspace_dir}/devel:{env.get('CMAKE_PREFIX_PATH', '')}"
            env[
                "LD_LIBRARY_PATH"
            ] = f"{self.workspace_dir}/devel/lib:{env.get('LD_LIBRARY_PATH', '')}"
            env[
                "PYTHONPATH"
            ] = f"{self.workspace_dir}/devel/lib/python3/dist-packages:{env.get('PYTHONPATH', '')}"

        return env

    def _build_workspace(self):
        """Build the catkin workspace"""
        print("Building workspace...")
        env = self._setup_environment()

        cmake_args = [
            "catkin_make",
            "-DCMAKE_BUILD_TYPE=Release",
            "-DCUDA_CUDART_LIBRARY=/usr/local/cuda/lib64/libcudart.so",
        ]

        try:
            self._run_command(cmake_args, cwd=self.workspace_dir, env=env)
            print("Workspace built successfully")
        except subprocess.CalledProcessError as e:
            print(f"Build failed: {e}")
            sys.exit(1)

    def _launch_zed_camera(self):
        """Launch ZED camera based on ZED_OPTION"""
        env = self._setup_environment()

        if self.zed_option:
            print("Launching ZED2i camera with RViz display...")
            launch_cmd = ["roslaunch", "zed_display_rviz", "display_zed2i.launch"]
        else:
            print("Launching ZED2i camera...")
            launch_cmd = ["roslaunch", "--wait", "zed_wrapper", "zed2i.launch"]

        try:
            self._run_command(launch_cmd, env=env, cwd=self.workspace_dir)
        except subprocess.CalledProcessError as e:
            print(f"Failed to launch ZED camera: {e}")
            sys.exit(1)

    def main(self):
        """Main execution function"""
        print("Starting ZED camera entrypoint...")
        print(f"ZED_OPTION: {self.zed_option}")

        try:
            # Build workspace
            self._build_workspace()

            # Launch ZED camera
            self._launch_zed_camera()

        except KeyboardInterrupt:
            print("\nCamera entrypoint interrupted")
        except Exception as e:
            print(f"Camera entrypoint failed: {e}")
            sys.exit(1)


if __name__ == "__main__":
    entrypoint = CameraEntrypoint()
    entrypoint.main()
