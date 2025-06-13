import os
import random
import socket
import string
import subprocess
import uuid
from typing import Any, Dict, List

import docker

# --------------------------------------------------------------------------- #
#  Repo mount + Tailscale basics
# --------------------------------------------------------------------------- #
REPO_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Pre-defined docker compose configurations shipped with the server. Users
# can only choose among these files and cannot supply their own.  The keys of
# this dict are short names exposed via the API/CLI and the values are the
# absolute paths to the corresponding compose YAML files.
COMPOSE_FILES: Dict[str, str] = {
    "openssh": os.path.join(os.path.dirname(__file__), "compose", "openssh.yaml"),
}

# For each compose file we also specify which service represents the main
# container that users will interact with (used to fetch the container ID after
# `docker compose` brings the stack up).
COMPOSE_MAIN_SERVICE: Dict[str, str] = {
    "openssh": "ssh",
}

TAILSCALE_IP = "100.76.98.95"
TAILSCALE_FUNNEL_DOMAIN = (
    "cesar-rog-zephyrus-g16-gu603vi-gu603vi.tail680469.ts.net"
)

# --------------------------------------------------------------------------- #
#  Helper: expose a local TCP port at /<container_id> via public HTTPS
# --------------------------------------------------------------------------- #
def expose_with_funnel_http_path(container_id: str, port: int) -> bool:
    """
    tailscale serve --bg --set-path /<container_id> tcp://localhost:<port>
    """
    try:
        subprocess.run(
            [
                "sudo", "tailscale", "serve",
                "--bg", "--set-path", f"/{container_id}",
                f"tcp://localhost:{port}",
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return True
    except subprocess.CalledProcessError as e:
        print(f"[tailscale serve] {e}")
        return False
# --------------------------------------------------------------------------- #
#  Docker Manager
# --------------------------------------------------------------------------- #
class DockerManager:
    """Create, configure, and remove SSH-enabled Docker containers."""

    def __init__(self) -> None:
        self.client = docker.from_env()

    # ----------------------------- utilities -------------------------------- #
    @staticmethod
    def _free_port() -> int:
        s = socket.socket()
        s.bind(("", 0))
        port = s.getsockname()[1]
        s.close()
        return port

    # ----------------------- image discovery helpers ----------------------- #
    def list_available_images(self) -> List[Dict[str, Any]]:
        images: List[Dict[str, Any]] = []
        for img in self.client.images.list():
            if img.tags:  # skip dangling layers
                for tag in img.tags:
                    images.append(
                        {
                            "id": img.id[:12],
                            "tag": tag,
                            "size": img.attrs.get("Size", 0),
                            "created": img.attrs.get("Created", ""),
                        }
                    )
        return sorted(images, key=lambda x: x["tag"])

    def list_available_compose(self) -> List[str]:
        """Return the list of supported docker-compose configurations."""
        return sorted(COMPOSE_FILES.keys())

    def get_image_info(self, image_tag: str) -> Dict[str, Any]:
        try:
            img = self.client.images.get(image_tag)
            return {
                "id": img.id,
                "tags": img.tags,
                "size": img.attrs.get("Size", 0),
                "created": img.attrs.get("Created", ""),
                "architecture": img.attrs.get("Architecture", ""),
                "os": img.attrs.get("Os", ""),
            }
        except docker.errors.ImageNotFound:
            return {}

    # --------------------- image-specific configuration -------------------- #
    @staticmethod
    def _get_image_environment(image_tag: str) -> Dict[str, str]:
        default_env = {
            "PASSWORD_ACCESS": "true",
            "USER_NAME": "dev",
            "PUID": "1000",
            "PGID": "1000",
            "TZ": "UTC",
        }

        if "hydrus" in image_tag.lower():
            return {
                "ROS_MASTER_URI": "http://host.docker.internal:11311",
                "ARDUINO_BOARD": "arduino:avr:uno",
                "DEPLOY": "false",
                "VOLUME": "true",
                "DISPLAY": os.environ.get("DISPLAY", ":0"),
                "SSH_ENABLE_PASSWORD_AUTH": "true",
                "SSH_ENABLE_ROOT": "true",
            }

        return default_env

    def _get_image_ports(self, image_tag: str) -> Dict[str, int]:
        p = self._free_port()
        if "hydrus" in image_tag.lower():
            return {"22/tcp": p, "8000/tcp": self._free_port(), "5000/tcp": self._free_port()}
        if "openssh-server" in image_tag.lower():
            return {"2222/tcp": p}
        return {"22/tcp": p}

    # ---------------- SSH bootstrap for Hydrus containers ------------------ #
    def _setup_ssh_in_container(self, container, password: str, image_tag: str) -> bool:
        if "hydrus" not in image_tag.lower():
            return True  # nothing to do for other images

        try:
            cmds = [
                "apt-get update -qq",
                "DEBIAN_FRONTEND=noninteractive apt-get install -y -qq openssh-server",
                "mkdir -p /var/run/sshd /root/.ssh",
                f"echo 'root:{password}' | chpasswd",
                # create dev user only if missing
                "id -u dev >/dev/null 2>&1 || useradd -m dev",
                f"echo 'dev:{password}' | chpasswd",
                # write a small override file instead of sed-ing the main config
                'printf "PermitRootLogin yes\\nPasswordAuthentication yes\\n" '
                '> /etc/ssh/sshd_config.d/99-custom.conf',
                "chmod 644 /etc/ssh/sshd_config.d/99-custom.conf",
                "systemctl enable ssh || true",
                "service ssh restart || service ssh start",
            ]

            for cmd in cmds:
                res = container.exec_run(cmd, user="root")
                if res.exit_code != 0:
                    print(f"[!] SSH setup failed at: {cmd}\n{res.output.decode()}")
                    return False
            return True
        except Exception as exc:
            print(f"[SSH setup exception] {exc}")
            return False

    # --------------------------- public API -------------------------------- #
    def create_container(self, user_id: int, image_tag: str = "linuxserver/openssh-server"):
        """Create container, keep it alive, expose SSH via Funnel."""
        cid = uuid.uuid4().hex[:8]
        password = "".join(random.choices(string.ascii_letters + string.digits, k=10))

        env = self._get_image_environment(image_tag)
        ports = self._get_image_ports(image_tag)
        if "openssh-server" in image_tag.lower():
            env["USER_PASSWORD"] = password

        volumes = {REPO_PATH: {"bind": "/workspace", "mode": "rw"}}
        if "hydrus" in image_tag.lower():
            volumes.update({
                "/tmp/.X11-unix": {"bind": "/tmp/.X11-unix", "mode": "rw"},
                f"{REPO_PATH}/rosbags": {"bind": "/rosbags", "mode": "rw"},
                f"{REPO_PATH}/yolo_models": {"bind": "/yolo_models", "mode": "rw"},
            })

        cfg: Dict[str, Any] = dict(
            image=image_tag,
            detach=True,
            ports=ports,
            environment=env,
            volumes=volumes,
            name=f"user-{user_id}-{cid}",
        )

        # Hydrus needs privileged + TTY
        if "hydrus" in image_tag.lower():
            cfg.update(privileged=True, stdin_open=True, tty=True)
        else:
            # For “stateless” images (Ubuntu, ROS, etc.) keep them running
            cfg["command"] = ["sleep", "infinity"]

        try:
            container = self.client.containers.run(**cfg)
            ssh_ready = self._setup_ssh_in_container(container, password, image_tag)

            main_port = next(iter(ports.values()))
            funnel_ok = expose_with_funnel_http_path(cid, main_port)
            funnel_path = f"/{cid}" if funnel_ok else None

            access_info = {
                "ssh_ready": ssh_ready,
                "tailscale_ip": TAILSCALE_IP,
                "funnel_domain": TAILSCALE_FUNNEL_DOMAIN,
                "funnel_path": funnel_path,
                "funnel_port": main_port if funnel_ok else None,
                "main_port": main_port,
                "all_ports": ports,
            }

            return cid, container.id, main_port, password, image_tag, access_info

        except docker.errors.ImageNotFound:
            raise ValueError(f"Docker image '{image_tag}' not found.")
        except Exception as exc:
            raise RuntimeError(f"Container creation failed: {exc}")

    def create_container_from_compose(self, user_id: int, compose_name: str):
        """Create a container using one of the bundled docker-compose files."""
        if compose_name not in COMPOSE_FILES:
            raise ValueError(f"Unknown compose config '{compose_name}'")

        compose_file = COMPOSE_FILES[compose_name]
        main_service = COMPOSE_MAIN_SERVICE[compose_name]

        cid = uuid.uuid4().hex[:8]
        project_name = f"user-{user_id}-{cid}"
        password = "".join(random.choices(string.ascii_letters + string.digits, k=10))

        try:
            subprocess.run(
                ["docker", "compose", "-f", compose_file, "-p", project_name, "up", "-d"],
                check=True,
                cwd=os.path.dirname(compose_file),
            )

            # obtain docker ID of the main service container
            result = subprocess.run(
                ["docker", "compose", "-f", compose_file, "-p", project_name, "ps", "-q", main_service],
                check=True,
                cwd=os.path.dirname(compose_file),
                capture_output=True,
                text=True,
            )
            docker_id = result.stdout.strip()

            container = self.client.containers.get(docker_id)
            ssh_ready = self._setup_ssh_in_container(container, password, compose_name)

            # fetch the mapped SSH port (randomly assigned by compose)
            ports = container.attrs.get("NetworkSettings", {}).get("Ports", {})
            ssh_ports = ports.get("22/tcp") or ports.get("2222/tcp")
            if ssh_ports:
                main_port = int(ssh_ports[0]["HostPort"])
            else:
                main_port = None

            funnel_path = None
            funnel_port = None
            if main_port:
                funnel_ok = expose_with_funnel_http_path(cid, main_port)
                if funnel_ok:
                    funnel_path = f"/{cid}"
                    funnel_port = main_port

            access_info = {
                "ssh_ready": ssh_ready,
                "tailscale_ip": TAILSCALE_IP,
                "funnel_domain": TAILSCALE_FUNNEL_DOMAIN,
                "funnel_path": funnel_path,
                "funnel_port": funnel_port,
                "main_port": main_port,
                "all_ports": ports,
            }

            return cid, docker_id, main_port, password, compose_name, access_info

        except subprocess.CalledProcessError as exc:
            raise RuntimeError(f"docker compose failed: {exc}")

    def remove_container(self, docker_id: str) -> None:
        try:
            cont = self.client.containers.get(docker_id)
            cont.stop()
            cont.remove()
        except docker.errors.NotFound:
            pass
