"""
Docker container management for Hydrus Docker deployment
"""

import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

from system_detector import SystemDetector
from vscode_integration import VSCodeIntegration


class DockerManager:
    """Manages Docker operations and container lifecycle"""

    def __init__(self, script_dir: Path, parent_dir: Path, rosbag_dir: Path):
        self.script_dir = script_dir
        self.parent_dir = parent_dir
        self.rosbag_dir = rosbag_dir

    def check_rosbags(self, auto_download: bool = False) -> bool:
        """Check if rosbags exist and optionally download if not"""
        # Check if directory exists and has any .bag files
        if not self.rosbag_dir.exists() or not list(self.rosbag_dir.glob("*.bag")):
            print(f"No ROS bag files found in {self.rosbag_dir}")

            if auto_download:
                print("Auto-downloading rosbag files...")
                try:
                    result = subprocess.run(
                        [
                            "python3",
                            str(self.parent_dir / "scripts/download_rosbag.py"),
                        ],
                        check=True,
                    )
                    return result.returncode == 0
                except subprocess.CalledProcessError:
                    print("Failed to download rosbag files")
                    return False
            else:
                print(
                    "Rosbag playback requested but no rosbag files found. Use --download-rosbag to download automatically."
                )
                return False
        return True

    def run_docker_compose(self, compose_file: str, config: dict, detach: bool = False):
        """Run docker compose with given configuration"""
        env = os.environ.copy()
        env.update(
            {
                "DEPLOY": str(config["deploy"]).lower(),
                "VOLUME": str(config["volume"]).lower(),
                "ZED_OPTION": str(config["zed_option"]).lower(),
                "RVIZ": str(config["rviz"]).lower(),
                "ROSBAG_PLAYBACK": str(config["rosbag_playback"]).lower(),
                "DEBUG_ARDUINO": str(config["debug_arduino"]).lower(),
                "TEST": str(config.get("test", False)).lower(),
            }
        )

        cmd = ["docker", "compose", "-f", compose_file, "up"]
        if config.get("test", False):
            cmd.append("--abort-on-container-exit")
        elif detach:
            cmd.append("-d")  # Add detached mode flag

        try:
            print(f"Starting Docker containers with {compose_file}...")
            print(
                f"🐳 Configuration: {', '.join([f'{k}={v}' for k, v in config.items() if v])}"
            )

            if detach:
                # Detached mode: run in background like the old behavior
                print("🔗 Running in detached mode (background)")
                print(f"💻 Executing: {' '.join(cmd)}")

                subprocess.run(cmd, env=env, check=True, cwd=self.script_dir)

                print("✅ Docker containers started successfully!")

                # Setup VS Code integration if requested
                if config.get("vscode", False):
                    VSCodeIntegration.setup_vscode_integration(
                        compose_file,
                        self.script_dir,
                        self.parent_dir,
                        config.get("install_vscode_extensions", False),
                    )

                print("\n" + "=" * 50)
                print("🐳 Docker Environment Ready!")
                print("=" * 50)
                print(f"Containers are running with configuration: {compose_file}")
                print("\nUseful commands:")
                print("• View running containers: docker ps")
                print("• View logs: docker compose logs -f")
                print("• Enter container: python3 run_docker.py --exec")
                print("• Stop containers: python3 run_docker.py --destroy")
                print("=" * 50)
            else:
                # Direct mode: replace terminal process (new default behavior)
                print(f"💻 Executing: {' '.join(cmd)}")
                print("🔄 Replacing current process with docker compose...")
                print("📋 All container logs will be shown directly in this terminal")
                print("🛑 Press Ctrl+C to stop containers and exit")
                print("=" * 60)

                # Change to script directory and replace current process with docker compose
                os.chdir(self.script_dir)
                os.execvpe("docker", cmd, env)

        except subprocess.CalledProcessError as e:
            print(f"❌ Docker compose failed with exit code {e.returncode}")
            sys.exit(e.returncode)
        except Exception as e:
            print(f"❌ Docker compose failed: {e}")
            sys.exit(1)

    def run_jetson_deployment(self):
        """Run Jetson-specific deployment"""
        print("Running Jetson deployment...")
        jetson_script = self.parent_dir / "scripts/jetson.py"
        if jetson_script.exists():
            subprocess.run([sys.executable, str(jetson_script)], check=True)
        else:
            # Fallback to bash script if Python version doesn't exist yet
            jetson_script = self.parent_dir / "scripts/jetson.sh"
            os.chmod(jetson_script, 0o755)
            subprocess.run([str(jetson_script)], check=True)

    def destroy_containers(self, compose_file: str):
        """Destroy containers created by Docker Compose"""
        try:
            print(f"🗑️  Destroying containers from {compose_file}...")
            print("🔄 Replacing current process with docker compose down...")

            cmd = [
                "docker",
                "compose",
                "-f",
                compose_file,
                "down",
                "--volumes",
                "--remove-orphans",
            ]
            env = os.environ.copy()

            # Change to script directory and replace current process
            os.chdir(self.script_dir)
            os.execvpe("docker", cmd, env)

        except Exception as e:
            print(f"❌ Failed to destroy containers: {e}")
            sys.exit(1)

    def exec_into_container(self, compose_file: str):
        """Execute into the running container and replace current shell"""
        try:
            print("🔍 Finding running Hydrus container...")

            # Get container information
            container_info = VSCodeIntegration.get_container_info(
                compose_file, self.script_dir
            )
            if not container_info:
                print("❌ No running Hydrus container found!")
                print("💡 Start containers first with: python3 run_docker.py")
                sys.exit(1)

            container_name = container_info["name"]

            if "running" not in container_info["status"].lower():
                print(
                    f"❌ Container {container_name} is not running (status: {container_info['status']})"
                )
                print("💡 Start containers first with: python3 run_docker.py")
                sys.exit(1)

            print(f"✅ Found running container: {container_name}")
            print(f"🚀 Executing into container...")

            # Set up the environment for the container shell
            container_env = os.environ.copy()
            container_env["TERM"] = "xterm-256color"  # Better terminal support

            # Use os.execvp to replace the current process with docker exec
            exec_cmd = [
                "docker",
                "exec",
                "-it",
                "--workdir",
                "/catkin_ws/src/hydrus-software-stack",
                container_name,
                "/bin/bash",
            ]

            print(f"💻 Executing: {' '.join(exec_cmd)}")
            print("🔄 Replacing current shell with container shell...")

            # Replace current process with docker exec
            os.execvpe("docker", exec_cmd, container_env)

        except Exception as e:
            print(f"❌ Error executing into container: {e}")
            sys.exit(1)
