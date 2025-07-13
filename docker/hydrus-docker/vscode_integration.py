"""
VS Code integration utilities for Hydrus Docker deployment
"""

import subprocess
import time


class VSCodeIntegration:
    """Handles VS Code integration and container attachment"""

    @staticmethod
    def check_vscode_installed() -> bool:
        """Check if VS Code is installed"""
        try:
            result = subprocess.run(
                ["code", "--version"], capture_output=True, text=True
            )
            return result.returncode == 0
        except FileNotFoundError:
            return False

    @staticmethod
    def check_vscode_extensions() -> dict:
        """Check if required VS Code extensions are installed"""
        required_extensions = {
            "ms-vscode-remote.remote-containers": "Dev Containers",
            "ms-vscode-remote.vscode-remote-extensionpack": "Remote Development",
        }

        installed_extensions = {}

        try:
            result = subprocess.run(
                ["code", "--list-extensions"], capture_output=True, text=True
            )
            if result.returncode == 0:
                installed = result.stdout.strip().split("\n")
                for ext_id, ext_name in required_extensions.items():
                    installed_extensions[ext_id] = {
                        "name": ext_name,
                        "installed": ext_id in installed,
                    }
            return installed_extensions
        except FileNotFoundError:
            return {}

    @staticmethod
    def install_vscode_extensions(extensions: dict) -> bool:
        """Install missing VS Code extensions"""
        missing_extensions = [
            ext_id for ext_id, info in extensions.items() if not info["installed"]
        ]

        if not missing_extensions:
            return True

        print("Installing missing VS Code extensions...")
        for ext_id in missing_extensions:
            try:
                print(f"Installing {extensions[ext_id]['name']}...")
                result = subprocess.run(
                    ["code", "--install-extension", ext_id],
                    capture_output=True,
                    text=True,
                )
                if result.returncode == 0:
                    print(f"✅ Successfully installed {extensions[ext_id]['name']}")
                else:
                    print(f"❌ Failed to install {extensions[ext_id]['name']}")
                    return False
            except Exception as e:
                print(f"❌ Error installing {ext_id}: {e}")
                return False

        return True

    @staticmethod
    def open_vscode_container(container_info, parent_dir) -> bool:
        """Open VS Code and attach to the running container"""
        try:
            container_name = container_info.name
            container_id = container_info.id

            print("🚀 Opening VS Code and connecting to container:")
            print(f"   Name: {container_name}")
            print(f"   ID: {container_id}")
            print(f"   Service: {container_info.service}")

            # Method 1: Try using the Dev Container extension with container ID
            print("📝 Attempting VS Code connection method 1 (Dev Container)...")
            try:
                result = subprocess.run(
                    [
                        "code",
                        "--folder-uri",
                        f"vscode-remote://attached-container+{container_id}/catkin_ws/src/hydrus-software-stack",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=10,
                )

                if result.returncode == 0:
                    print("✅ VS Code opened successfully using Dev Container method")
                    return True
                else:
                    print(
                        f"⚠️  Dev Container method failed (exit code: {result.returncode})"
                    )
            except subprocess.TimeoutExpired:
                print("⚠️  Dev Container method timed out")
            except Exception as e:
                print(f"⚠️  Dev Container method error: {e}")

            # Method 2: Try opening the project directory
            print("📝 Attempting VS Code connection method 2 (Project directory)...")
            try:
                subprocess.Popen(
                    ["code", str(parent_dir)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )

                print("✅ VS Code opened with project directory")
                print("💡 To connect to the container manually:")
                print("   1. Press Ctrl+Shift+P")
                print("   2. Type 'Dev Containers: Attach to Running Container'")
                print(f"   3. Select container: {container_name}")
                print("   4. Open folder: /catkin_ws/src/hydrus-software-stack")
                return True

            except Exception as e:
                print(f"❌ Failed to open VS Code project directory: {e}")

            return False

        except Exception as e:
            print(f"❌ Failed to open VS Code: {e}")
            return False

    @staticmethod
    def setup_vscode_integration(
        compose_file: str, script_dir, parent_dir, auto_install_extensions: bool = False
    ) -> bool:
        """Setup and launch VS Code integration if available"""
        from .docker_manager import DockerManager

        # Check if VS Code is installed
        if not VSCodeIntegration.check_vscode_installed():
            print("VS Code not found. Skipping VS Code integration.")
            return False

        print("VS Code detected. Checking extensions...")

        # Check extensions
        extensions = VSCodeIntegration.check_vscode_extensions()
        if not extensions:
            print("Could not check VS Code extensions. Skipping VS Code integration.")
            return False

        # Check if required extensions are installed
        missing_extensions = [
            info["name"] for info in extensions.values() if not info["installed"]
        ]

        if missing_extensions:
            print(f"Missing required extensions: {', '.join(missing_extensions)}")

            if auto_install_extensions:
                print("Auto-installing missing extensions...")
                if not VSCodeIntegration.install_vscode_extensions(extensions):
                    print("Failed to install extensions. Skipping VS Code integration.")
                    return False
            else:
                print(
                    "Use --install-vscode-extensions to automatically install missing extensions."
                )
                print("Skipping VS Code integration without required extensions.")
                return False

        # Wait a moment for container to be fully ready
        print("Waiting for container to be ready...")
        time.sleep(3)

        # Get container information using DockerManager
        docker_manager = DockerManager(script_dir)
        container_info = docker_manager.get_container_info(compose_file, script_dir)
        if not container_info:
            print(
                "Could not find running Hydrus container. Skipping VS Code integration."
            )
            return False

        if "running" not in container_info.status.lower():
            print(
                f"Container is not running (status: {container_info.status}). Skipping VS Code integration."
            )
            return False

        # Open VS Code
        return VSCodeIntegration.open_vscode_container(container_info, parent_dir)
