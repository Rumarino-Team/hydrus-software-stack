"""
Docker container management for Hydrus Docker deployment
"""

import json
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass
class DockerConfig:
    """Docker configuration data class"""

    volume: bool = False
    force_cpu: bool = False
    force_jetson: bool = False
    vscode: bool = False
    install_vscode_extensions: bool = False


@dataclass
class ContainerInfo:
    """Container information data class"""

    name: str
    id: str
    status: str
    service: str


class DockerManager:
    """Manages Docker operations and container lifecycle"""

    def __init__(self, script_dir=None):
        self.script_dir = script_dir or Path(__file__).parent.absolute()
        # Docker compose files are in the parent directory (docker/)
        self.docker_dir = Path(__file__).parent.parent.absolute()

    def run_docker_compose(
        self, compose_file: str, config: DockerConfig, detach: bool = False
    ):
        """Run docker compose with given configuration"""
        cmd = ["docker", "compose", "-f", compose_file, "up"]

        try:
            print(f"Starting Docker containers with {compose_file}...")
            print(f"🐳 Configuration: {config}")

            if detach:
                # Detached mode: run in background like the old behavior
                print("🔗 Running in detached mode (background)")
                print(f"💻 Executing: {' '.join(cmd)}")

                subprocess.run(cmd, check=True, cwd=self.docker_dir)

                print("✅ Docker containers started successfully!")
            else:
                # Direct mode: replace terminal process (new default behavior)
                print(f"💻 Executing: {' '.join(cmd)}")
                print("🔄 Replacing current process with docker compose...")
                print("📋 All container logs will be shown directly in this terminal")
                print("🛑 Press Ctrl+C to stop containers and exit")
                print("=" * 60)

                # Change to docker directory and replace current process with docker compose
                os.chdir(self.docker_dir)
                os.execvpe("docker", cmd, env=os.environ.copy())

        except subprocess.CalledProcessError as e:
            print(f"❌ Docker compose failed with exit code {e.returncode}")
            sys.exit(e.returncode)
        except Exception as e:
            print(f"❌ Docker compose failed: {e}")
            sys.exit(1)

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

            # Change to docker directory and replace current process
            os.chdir(self.docker_dir)
            os.execvpe("docker", cmd, env)

        except Exception as e:
            print(f"❌ Failed to destroy containers: {e}")
            sys.exit(1)

    def get_container_info(
        self, compose_file: str, script_dir=None
    ) -> Optional[ContainerInfo]:
        """Get information about the running container"""
        # Use docker_dir for compose operations
        work_dir = script_dir if script_dir else self.docker_dir
        try:
            print("🔍 Detecting running containers...")

            # Method 1: Use docker compose ps with specific format to get actual container IDs
            result = subprocess.run(
                [
                    "docker",
                    "compose",
                    "-f",
                    compose_file,
                    "ps",
                    "--format",
                    "table {{.Name}}\t{{.ID}}\t{{.Service}}\t{{.Status}}",
                ],
                capture_output=True,
                text=True,
                cwd=work_dir,
            )

            if result.returncode == 0:
                print("📋 Docker compose output received, parsing containers...")
                lines = result.stdout.strip().split("\n")

                # Skip header line if present
                data_lines = [
                    line
                    for line in lines
                    if not line.startswith("NAME") and line.strip()
                ]

                for line in data_lines:
                    if line.strip():
                        parts = line.split("\t")
                        if len(parts) >= 4:
                            container_name = parts[0].strip()
                            container_id = parts[1].strip()
                            service_name = parts[2].strip()
                            status = parts[3].strip()

                            print(
                                f"   Found container: {container_name} (service: {service_name}, status: {status}, id: {container_id})"
                            )

                            # Look for hydrus-related services
                            if any(
                                keyword in service_name.lower()
                                for keyword in ["hydrus", "hydrus_cpu", "hydrus_cuda"]
                            ):
                                print(
                                    f"✅ Found Hydrus container: {container_name} with ID: {container_id}"
                                )
                                return ContainerInfo(
                                    name=container_name,
                                    id=container_id,
                                    status=status,
                                    service=service_name,
                                )

            # Method 2: Fallback to JSON format and extract from container list
            print("🔄 Fallback method 1: Using docker compose ps JSON format...")
            result = subprocess.run(
                ["docker", "compose", "-f", compose_file, "ps", "--format", "json"],
                capture_output=True,
                text=True,
                cwd=work_dir,
            )

            if result.returncode == 0:
                for line in result.stdout.strip().split("\n"):
                    if line.strip():
                        try:
                            container = json.loads(line)
                            service_name = container.get("Service", "")
                            container_name = container.get("Name", "")

                            # Look for hydrus-related services
                            if any(
                                keyword in service_name.lower()
                                for keyword in ["hydrus", "hydrus_cpu", "hydrus_cuda"]
                            ):
                                # Get the actual container ID by querying docker directly with the container name
                                id_result = subprocess.run(
                                    [
                                        "docker",
                                        "inspect",
                                        "--format",
                                        "{{.Id}}",
                                        container_name,
                                    ],
                                    capture_output=True,
                                    text=True,
                                )

                                if id_result.returncode == 0:
                                    actual_container_id = id_result.stdout.strip()[
                                        :12
                                    ]  # Use short ID
                                    state = container.get("State", "")

                                    print(
                                        f"✅ Found Hydrus container via JSON: {container_name} with actual ID: {actual_container_id}"
                                    )
                                    return ContainerInfo(
                                        name=container_name,
                                        id=actual_container_id,
                                        status=state,
                                        service=service_name,
                                    )
                        except json.JSONDecodeError:
                            print(f"   ⚠️  JSON decode error for line: {line[:100]}...")
                            continue

            # Method 3: Direct docker ps fallback
            print("🔄 Fallback method 2: Using direct docker ps...")
            result = subprocess.run(
                [
                    "docker",
                    "ps",
                    "--format",
                    "{{.Names}}\t{{.ID}}\t{{.Status}}\t{{.Image}}",
                ],
                capture_output=True,
                text=True,
            )

            if result.returncode == 0:
                for line in result.stdout.strip().split("\n"):
                    if line.strip():
                        parts = line.split("\t")
                        if len(parts) >= 4:
                            name, container_id, status, image = (
                                parts[0],
                                parts[1],
                                parts[2],
                                parts[3],
                            )
                            print(
                                f"   Found container: {name} (image: {image}, id: {container_id})"
                            )

                            # Look for hydrus in name or image
                            if any(
                                keyword in name.lower()
                                for keyword in ["hydrus", "hydrus_cpu", "hydrus_cuda"]
                            ) or any(
                                keyword in image.lower()
                                for keyword in ["hydrus", "docker-hydrus"]
                            ):
                                print(
                                    f"✅ Found Hydrus container via direct docker ps: {name} with ID: {container_id}"
                                )
                                return ContainerInfo(
                                    name=name,
                                    id=container_id,
                                    status=status,
                                    service="hydrus",
                                )

            print("❌ No Hydrus containers found")

        except Exception as e:
            print(f"❌ Error getting container info: {e}")

        return None

    def exec_into_container(
        self,
        compose_file: str,
        command: Optional[list] = None,
        interactive: bool = False,
    ):
        """Execute into the running container and replace current shell or run specific command"""
        try:
            print("🔍 Finding running Hydrus container...")

            # Get container information
            container_info = self.get_container_info(compose_file)
            if not container_info:
                print("❌ No running Hydrus container found!")
                print("💡 Start containers first with: python3 run_docker.py")
                sys.exit(1)

            container_name = container_info.name

            if "running" not in container_info.status.lower():
                print(
                    f"❌ Container {container_name} is not running (status: {container_info.status})"
                )
                print("💡 Start containers first with: python3 run_docker.py")
                sys.exit(1)

            print(f"✅ Found running container: {container_name}")

            # Set up the environment for the container shell
            container_env = os.environ.copy()
            container_env["TERM"] = "xterm-256color"  # Better terminal support

            if interactive or (command is None or len(command) == 0):
                # Interactive shell mode
                print("🚀 Executing into container (interactive shell)...")

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
            else:
                # Execute specific command
                print(f"🚀 Executing command in container: {' '.join(command)}")

                exec_cmd = [
                    "docker",
                    "exec",
                    "-it",
                    "--workdir",
                    "/catkin_ws/src/hydrus-software-stack",
                    container_name,
                ] + command

                print(f"💻 Executing: {' '.join(exec_cmd)}")

                # Run the command and wait for completion
                result = subprocess.run(exec_cmd, env=container_env)
                sys.exit(result.returncode)

        except Exception as e:
            print(f"❌ Error executing into container: {e}")
            sys.exit(1)
