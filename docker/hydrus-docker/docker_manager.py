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
        force_type: Optional[str] = None,
        dev_mode: bool = False,
    ):
        """Execute into the container, starting it if necessary

        Args:
            compose_file: Path to compose file
            command: Command to execute (None for interactive shell)
            interactive: Force interactive mode
            force_type: Force specific container type ("cpu" or "cuda")
            dev_mode: Use development working directory (/catkin_ws/src/hydrus-software-stack)
        """
        try:
            print("🔍 Finding Hydrus container...")

            container_info = None

            # If forcing a specific type, use the new method
            if force_type:
                container_info = self.force_container_type(force_type, compose_file)
            else:
                # Use the original method
                container_info = self.get_container_info(compose_file)

                if not container_info:
                    print("❌ No Hydrus container found! Starting containers...")
                    self._start_containers_and_wait(compose_file)

                    # Try to get container info again after starting
                    container_info = self.get_container_info(compose_file)
                    if not container_info:
                        print("❌ Failed to start or find Hydrus container!")
                        sys.exit(1)

            if not container_info:
                print("❌ Failed to get container information!")
                sys.exit(1)

            container_name = container_info.name

            # Check if container is running
            if (
                "running" not in container_info.status.lower()
                and "up" not in container_info.status.lower()
            ):
                print(
                    f"⚠️  Container {container_name} is not running (status: {container_info.status})"
                )

                if force_type:
                    # Try to start the specific container
                    print(f"🚀 Starting {force_type.upper()} container...")
                    try:
                        subprocess.run(
                            ["docker", "start", container_name],
                            check=True,
                            capture_output=True,
                            text=True,
                        )
                        print(f"✅ Started {container_name}")
                    except subprocess.CalledProcessError:
                        print("❌ Failed to start container, creating new one...")
                        self._start_containers_and_wait(compose_file)
                else:
                    print("🚀 Starting containers...")
                    self._start_containers_and_wait(compose_file)

                # Verify container is now running
                if force_type:
                    updated_info = self.get_container_info_by_type(force_type)
                else:
                    updated_info = self.get_container_info(compose_file)

                if not updated_info or (
                    "running" not in updated_info.status.lower()
                    and "up" not in updated_info.status.lower()
                ):
                    print("❌ Failed to start containers!")
                    sys.exit(1)
                container_name = updated_info.name

            print(f"✅ Found running container: {container_name}")

            # Set up the environment for the container shell
            container_env = os.environ.copy()
            container_env["TERM"] = "xterm-256color"  # Better terminal support

            # Determine working directory based on dev_mode
            workdir = (
                "/home/catkin_ws/src/hydrus-software-stack"
                if dev_mode
                else "/catkin_ws/src/hydrus-software-stack"
            )
            workdir_desc = "development volume" if dev_mode else "workspace root"

            if interactive or (command is None or len(command) == 0):
                # Interactive shell mode
                print(
                    f"🚀 Executing into container (interactive shell in {workdir_desc})..."
                )

                # Use os.execvp to replace the current process with docker exec
                exec_cmd = [
                    "docker",
                    "exec",
                    "-it",
                    "--workdir",
                    workdir,
                    container_name,
                    "/bin/bash",
                ]

                print(f"💻 Executing: {' '.join(exec_cmd)}")
                print("🔄 Replacing current shell with container shell...")

                # Replace current process with docker exec
                os.execvpe("docker", exec_cmd, container_env)
            else:
                # Execute specific command
                print(
                    f"🚀 Executing command in container ({workdir_desc}): {' '.join(command)}"
                )

                exec_cmd = [
                    "docker",
                    "exec",
                    "-it",
                    "--workdir",
                    workdir,
                    container_name,
                ] + command

                print(f"💻 Executing: {' '.join(exec_cmd)}")

                # Run the command and wait for completion
                result = subprocess.run(exec_cmd, env=container_env)
                sys.exit(result.returncode)

        except Exception as e:
            print(f"❌ Error executing into container: {e}")
            sys.exit(1)

    def _start_containers_and_wait(self, compose_file: str):
        """Start containers and wait for them to be ready"""
        try:
            print("🚀 Starting Docker containers...")

            # Start containers in detached mode
            cmd = ["docker", "compose", "-f", compose_file, "up", "-d"]

            subprocess.run(
                cmd, check=True, cwd=self.docker_dir, capture_output=True, text=True
            )

            print("✅ Containers started successfully!")

            # Wait a moment for containers to initialize
            import time

            print("⏳ Waiting for containers to initialize...")
            time.sleep(5)

        except subprocess.CalledProcessError as e:
            print(f"❌ Failed to start containers: {e}")
            if e.stderr:
                print(f"Error output: {e.stderr}")
            sys.exit(1)
        except Exception as e:
            print(f"❌ Error starting containers: {e}")
            sys.exit(1)

    def get_container_info_by_type(
        self, container_type: str = "any"
    ) -> Optional[ContainerInfo]:
        """
        Get container information by type using docker ps -a (shows all containers)

        Args:
            container_type: "cpu", "cuda", or "any"
        """
        try:
            print(f"🔍 Looking for {container_type} containers using docker ps -a...")

            # Use docker ps -a to see ALL containers (running and stopped)
            result = subprocess.run(
                [
                    "docker",
                    "ps",
                    "-a",  # Show all containers, not just running ones
                    "--format",
                    "{{.Names}}\t{{.ID}}\t{{.Status}}\t{{.Image}}",
                ],
                capture_output=True,
                text=True,
            )

            if result.returncode != 0:
                print(f"❌ Failed to run docker ps -a: {result.stderr}")
                return None

            containers_found = []

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

                        # Check if this is a Hydrus container
                        is_hydrus = any(
                            keyword in name.lower()
                            for keyword in ["hydrus", "docker-hydrus"]
                        ) or any(
                            keyword in image.lower()
                            for keyword in ["hydrus", "docker-hydrus"]
                        )

                        if is_hydrus:
                            # Determine container type
                            is_cpu = any(
                                keyword in name.lower()
                                for keyword in ["hydrus_cpu", "hydrus-cpu", "cpu"]
                            )
                            is_cuda = any(
                                keyword in name.lower()
                                for keyword in ["hydrus_cuda", "hydrus-cuda", "cuda"]
                            )

                            container_info = {
                                "name": name,
                                "id": container_id,
                                "status": status,
                                "image": image,
                                "type": "cpu"
                                if is_cpu
                                else "cuda"
                                if is_cuda
                                else "unknown",
                            }

                            containers_found.append(container_info)
                            print(
                                f"   Found {container_info['type']} container: {name} (status: {status}, id: {container_id})"
                            )

            # Filter by requested type
            if container_type != "any":
                containers_found = [
                    c for c in containers_found if c["type"] == container_type
                ]

            if not containers_found:
                print(f"❌ No {container_type} Hydrus containers found")
                return None

            # Prefer running containers, then any container
            running_containers = [
                c for c in containers_found if "up" in c["status"].lower()
            ]

            if running_containers:
                container = running_containers[0]
                print(
                    f"✅ Found running {container['type']} container: {container['name']}"
                )
            else:
                container = containers_found[0]
                print(
                    f"✅ Found stopped {container['type']} container: {container['name']}"
                )

            # Determine service name based on container type
            service_name = (
                f"hydrus_{container['type']}"
                if container["type"] in ["cpu", "cuda"]
                else "hydrus"
            )

            return ContainerInfo(
                name=container["name"],
                id=container["id"],
                status=container["status"],
                service=service_name,
            )

        except Exception as e:
            print(f"❌ Error getting container info by type: {e}")
            return None

    def force_container_type(
        self, container_type: str, compose_file: str
    ) -> Optional[ContainerInfo]:
        """
        Force a specific container type to be running (CPU or CUDA)
        Stops conflicting containers and starts the requested type

        Args:
            container_type: "cpu" or "cuda"
            compose_file: Path to the appropriate compose file
        """
        try:
            print(f"🚀 Forcing {container_type.upper()} container type...")

            # Step 1: Check if the desired type is already running
            desired_container = self.get_container_info_by_type(container_type)

            if desired_container and "up" in desired_container.status.lower():
                print(
                    f"✅ {container_type.upper()} container already running: {desired_container.name}"
                )
                return desired_container

            # Step 2: Stop conflicting containers
            other_type = "cuda" if container_type == "cpu" else "cpu"
            conflicting_container = self.get_container_info_by_type(other_type)

            if conflicting_container and "up" in conflicting_container.status.lower():
                print(
                    f"🛑 Stopping conflicting {other_type.upper()} container: {conflicting_container.name}"
                )
                try:
                    subprocess.run(
                        ["docker", "stop", conflicting_container.name],
                        check=True,
                        capture_output=True,
                        text=True,
                    )
                    print(f"✅ Stopped {conflicting_container.name}")
                except subprocess.CalledProcessError as e:
                    print(f"⚠️  Failed to stop {conflicting_container.name}: {e}")

            # Step 3: Start or create the desired container
            if desired_container:
                # Container exists but is stopped, start it
                print(
                    f"🚀 Starting existing {container_type.upper()} container: {desired_container.name}"
                )
                try:
                    subprocess.run(
                        ["docker", "start", desired_container.name],
                        check=True,
                        capture_output=True,
                        text=True,
                    )
                    print(f"✅ Started {desired_container.name}")

                    # Update status
                    updated_container = self.get_container_info_by_type(container_type)
                    return updated_container

                except subprocess.CalledProcessError as e:
                    print(
                        f"⚠️  Failed to start existing container, will create new one: {e}"
                    )

            # Step 4: Create new containers using compose file
            print(
                f"🏗️  Creating new {container_type.upper()} containers using {compose_file}"
            )
            self._start_containers_and_wait(compose_file)

            # Step 5: Get the newly created container info
            new_container = self.get_container_info_by_type(container_type)
            if new_container:
                print(
                    f"✅ Successfully created and started {container_type.upper()} container: {new_container.name}"
                )
                return new_container
            else:
                print(f"❌ Failed to create {container_type.upper()} container")
                return None

        except Exception as e:
            print(f"❌ Error forcing {container_type} container type: {e}")
            return None
