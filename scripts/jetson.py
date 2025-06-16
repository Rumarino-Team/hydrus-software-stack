#!/usr/bin/env python3
"""
Hydrus Jetson Deployment Script
Replaces jetson.sh with improved modularity and error handling
"""

import os
import platform
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional


class JetsonDeploymentManager:
    def __init__(self):
        self.network_name = "ros-network"
        self.ros_master_uri = "http://ros-master:11311"
        self.use_qemu = False
        self.qemu_args = []
        self.zed_option = False
        self.deploy = True

        # Detect architecture and setup QEMU if needed
        self._setup_architecture()

    def _run_command(
        self,
        cmd: List[str],
        check: bool = True,
        capture_output: bool = False,
        env: Optional[Dict] = None,
    ) -> subprocess.CompletedProcess:
        """Run a command with proper error handling"""
        if env is None:
            env = os.environ.copy()

        try:
            return subprocess.run(
                cmd, check=check, capture_output=capture_output, env=env, text=True
            )
        except subprocess.CalledProcessError as e:
            if check:
                print(f"Command failed: {' '.join(cmd)}")
                print(f"Exit code: {e.returncode}")
                if capture_output and e.stderr:
                    print(f"Error: {e.stderr}")
            raise

    def _setup_architecture(self):
        """Detect system architecture and set up QEMU if needed"""
        arch = platform.machine()

        if arch == "x86_64":
            print("Detected x86_64 architecture. Will use QEMU for ARM emulation.")

            # Check if QEMU is installed
            try:
                self._run_command(
                    ["qemu-system-aarch64", "--version"], capture_output=True
                )
            except (subprocess.CalledProcessError, FileNotFoundError):
                print("ERROR: QEMU is not installed. Please install it with:")
                print(
                    "  sudo apt-get install qemu-system-arm qemu-efi qemu-user-static"
                )
                sys.exit(1)

            # Register QEMU binary formats if not already done
            binfmt_file = Path("/proc/sys/fs/binfmt_misc/qemu-aarch64")
            if not binfmt_file.exists():
                print("Setting up QEMU binary formats for ARM emulation...")
                try:
                    self._run_command(
                        [
                            "docker",
                            "run",
                            "--rm",
                            "--privileged",
                            "multiarch/qemu-user-static",
                            "--reset",
                            "-p",
                            "yes",
                        ]
                    )
                except subprocess.CalledProcessError as e:
                    print(f"Failed to setup QEMU binary formats: {e}")
                    sys.exit(1)

            self.use_qemu = True
            self.qemu_args = ["--platform", "linux/arm64"]
            print("QEMU emulation enabled for ARM containers.")

    def _create_docker_network(self):
        """Create custom Docker network if it doesn't exist"""
        try:
            # Check if network exists
            result = self._run_command(
                ["docker", "network", "ls", "--format", "{{.Name}}"],
                capture_output=True,
            )

            if self.network_name not in result.stdout:
                print(f"Creating custom Docker network: {self.network_name}")
                self._run_command(["docker", "network", "create", self.network_name])
            else:
                print(f"Docker network {self.network_name} already exists")

        except subprocess.CalledProcessError as e:
            print(f"Failed to create Docker network: {e}")
            sys.exit(1)

    def _container_exists(self, container_name: str) -> bool:
        """Check if a container exists"""
        try:
            result = self._run_command(
                ["docker", "ps", "-a", "--format", "{{.Names}}"], capture_output=True
            )
            return container_name in result.stdout.split("\n")
        except subprocess.CalledProcessError:
            return False

    def _container_running(self, container_name: str) -> bool:
        """Check if a container is running"""
        try:
            result = self._run_command(
                ["docker", "ps", "--format", "{{.Names}}"], capture_output=True
            )
            return container_name in result.stdout.split("\n")
        except subprocess.CalledProcessError:
            return False

    def _start_ros_master(self):
        """Run the ROS master container"""
        print("=" * 50)
        print("Step 1: Setting up ROS master container")
        print("=" * 50)

        if self._container_exists("ros-master"):
            if self._container_running("ros-master"):
                print("ROS master is already running.")
            else:
                print("Starting existing ROS master container...")
                self._run_command(["docker", "start", "ros-master"])
        else:
            print("Creating and starting ROS master container...")
            cmd = [
                "docker",
                "run",
                "-d",
                "--name",
                "ros-master",
                "--network",
                self.network_name,
                "-p",
                "11311:11311",
                "ros:melodic-ros-core",
                "stdbuf",
                "-o",
                "L",
                "roscore",
            ]
            self._run_command(cmd)

        # Wait for ROS master to be up
        print("Waiting for ROS master to start...")
        import time

        time.sleep(3)

    def _build_and_run_zed_camera(self):
        """Build and run the ZED camera container"""
        print("=" * 50)
        print("Step 2: Setting up ZED camera container")
        print("=" * 50)

        dockerfile_path = "docker/jetson/camera.Dockerfile"

        # Build ZED camera container
        if self.use_qemu:
            print("Building ZED camera container with QEMU emulation...")
            build_cmd = (
                ["docker", "build"]
                + self.qemu_args
                + ["-t", "zed-camera", "-f", dockerfile_path, "."]
            )
        else:
            print("Building ZED camera container...")
            build_cmd = [
                "docker",
                "build",
                "-t",
                "zed-camera",
                "-f",
                dockerfile_path,
                ".",
            ]

        self._run_command(build_cmd)

        # Run ZED camera container
        if self._container_exists("zed-camera"):
            if self._container_running("zed-camera"):
                print("ZED camera container is already running.")
            else:
                print("Starting existing ZED camera container...")
                self._run_command(["docker", "start", "zed-camera"])
        else:
            print("Creating and starting ZED camera container...")

            env_vars = {
                "ROS_MASTER_URI": "http://ros-master:11311",
                "ZED_OPTION": str(self.zed_option).lower(),
            }

            cmd = [
                "docker",
                "run",
                "-d",
                "--name",
                "zed-camera",
                "--network",
                self.network_name,
                "--privileged",
                "--gpus",
                "all",
            ]

            # Add environment variables
            for key, value in env_vars.items():
                cmd.extend(["--env", f"{key}={value}"])

            # Add QEMU args if needed
            if self.use_qemu:
                cmd.extend(self.qemu_args)

            cmd.append("zed-camera")
            self._run_command(cmd)

        # Wait for ZED camera to be ready
        print("Waiting for ZED camera to start...")
        import time

        time.sleep(3)

    def _build_and_run_hydrus(self):
        """Build and run the Hydrus container"""
        print("=" * 50)
        print("Step 3: Setting up Hydrus container")
        print("=" * 50)

        dockerfile_path = "docker/jetson/hydrus.Dockerfile"

        # Check if Hydrus image exists
        try:
            result = self._run_command(
                [
                    "docker",
                    "images",
                    "hydrus:latest",
                    "--format",
                    "{{.Repository}}:{{.Tag}}",
                ],
                capture_output=True,
            )

            if "hydrus:latest" not in result.stdout:
                print("Building Hydrus image...")
                if self.use_qemu:
                    build_cmd = (
                        ["docker", "build"]
                        + self.qemu_args
                        + ["-t", "hydrus:latest", "-f", dockerfile_path, "."]
                    )
                else:
                    build_cmd = [
                        "docker",
                        "build",
                        "-t",
                        "hydrus:latest",
                        "-f",
                        dockerfile_path,
                        ".",
                    ]

                self._run_command(build_cmd)
            else:
                print("Hydrus image already exists, skipping build.")
        except subprocess.CalledProcessError:
            print("Error checking Hydrus image, building anyway...")
            if self.use_qemu:
                build_cmd = (
                    ["docker", "build"]
                    + self.qemu_args
                    + ["-t", "hydrus:latest", "-f", dockerfile_path, "."]
                )
            else:
                build_cmd = [
                    "docker",
                    "build",
                    "-t",
                    "hydrus:latest",
                    "-f",
                    dockerfile_path,
                    ".",
                ]

            self._run_command(build_cmd)

        # Run Hydrus container
        if self._container_exists("hydrus"):
            if self._container_running("hydrus"):
                print("Hydrus container is already running.")
            else:
                print("Starting existing Hydrus container...")
                self._run_command(["docker", "start", "hydrus"])
        else:
            print("Creating and starting Hydrus container...")

            env_vars = {
                "ROS_MASTER_URI": "http://ros-master:11311",
                "ARDUINO_BOARD": "arduino:avr:mega",
                "DEPLOY": str(self.deploy).lower(),
            }

            cmd = [
                "docker",
                "run",
                "-d",
                "--name",
                "hydrus",
                "--network",
                self.network_name,
                "--privileged",
                "--gpus",
                "all",
                "-p",
                "8000:8000",
                "--device",
                "/dev/ttyACM0:/dev/ttyACM0",
            ]

            # Add environment variables
            for key, value in env_vars.items():
                cmd.extend(["--env", f"{key}={value}"])

            # Add QEMU args if needed
            if self.use_qemu:
                cmd.extend(self.qemu_args)

            cmd.extend(["-it", "hydrus:latest"])
            self._run_command(cmd)

    def main(self):
        """Main execution function"""
        print("Starting Jetson deployment...")

        # Set environment variables
        os.environ["ROS_MASTER_URI"] = self.ros_master_uri

        try:
            # Create Docker network
            self._create_docker_network()

            # Start ROS master
            self._start_ros_master()

            # Build and run ZED camera
            self._build_and_run_zed_camera()

            # Build and run Hydrus
            self._build_and_run_hydrus()

            print(f"Containers are up and running on network '{self.network_name}'!")

        except Exception as e:
            print(f"Deployment failed: {e}")
            sys.exit(1)


if __name__ == "__main__":
    manager = JetsonDeploymentManager()
    manager.main()
