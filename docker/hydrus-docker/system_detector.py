"""
System detection utilities for Hydrus Docker deployment
"""

import platform
import subprocess
from enum import Enum
from typing import Optional


class SystemType(Enum):
    """Enumeration for different system types"""

    JETSON_TX2 = "jetson_tx2"
    WSL = "wsl"
    NVIDIA_GPU = "nvidia_gpu"
    CPU = "cpu"


compose_files = {
    SystemType.JETSON_TX2: None,  # Jetson uses custom deployment
    SystemType.WSL: "docker-compose-amd64-cpu.yaml",
    SystemType.NVIDIA_GPU: "docker-compose-amd64-cuda.yaml",
    SystemType.CPU: "docker-compose-amd64-cpu.yaml",
}


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
    def determine_compose_file(
        system_type: Optional[SystemType] = None,
    ) -> Optional[str]:
        """Determine which compose file to use based on arguments and system detection"""
        if system_type:
            compose_file = compose_files.get(system_type)
            print(
                f"Using forced system type: {system_type.value}. Compose file: {compose_file}"
            )
            return compose_file
        else:
            # Auto-detect
            if SystemDetector.is_wsl():
                print("WSL detected. Using CPU deployment.")
                return compose_files.get(SystemType.CPU)
            elif SystemDetector.check_nvidia_gpu():
                print("GPU available. Using CUDA deployment.")
                return compose_files.get(SystemType.NVIDIA_GPU)
            else:
                print("No GPU available. Using CPU deployment.")
                return compose_files.get(SystemType.CPU)
