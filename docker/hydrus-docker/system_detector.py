"""
System detection utilities for Hydrus Docker deployment
"""

import platform
import subprocess
from typing import Optional


class SystemDetector:
    """Detects system capabilities and environment"""

    @staticmethod
    def is_jetson_tx2() -> bool:
        """Check if the system is Jetson TX2"""
        try:
            arch = platform.machine()
            if arch == "aarch64":
                with open("/etc/nv_tegra_release", "r") as f:
                    return True
        except FileNotFoundError:
            pass
        return False

    @staticmethod
    def is_wsl() -> bool:
        """Check if the system is running in WSL"""
        try:
            with open("/proc/version", "r") as f:
                return "microsoft" in f.read().lower()
        except FileNotFoundError:
            return False

    @staticmethod
    def check_nvidia_gpu() -> bool:
        """Check if NVIDIA GPUs are available"""
        try:
            subprocess.run(["nvidia-smi"], check=True, capture_output=True)
            return True
        except (subprocess.CalledProcessError, FileNotFoundError):
            return False

    @staticmethod
    def determine_compose_file(args) -> Optional[str]:
        """Determine which compose file to use based on arguments and system detection"""
        if args.force_cpu or getattr(args, "test", False):
            return "docker-compose-amd64-cpu.yaml"
        elif args.force_jetson:
            return None  # Jetson uses custom deployment
        elif args.cuda:
            return "docker-compose-amd64-cuda.yaml"
        else:
            # Auto-detect
            if SystemDetector.is_wsl():
                print("WSL detected. Using CPU deployment.")
                return "docker-compose-amd64-cpu.yaml"
            elif SystemDetector.check_nvidia_gpu():
                print("GPU available. Using CUDA deployment.")
                return "docker-compose-amd64-cuda.yaml"
            else:
                print("No GPU available. Using CPU deployment.")
                return "docker-compose-amd64-cpu.yaml"
