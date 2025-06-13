"""
VS Code integration utilities for Hydrus Docker deployment
"""

import subprocess
import json
import time
from typing import Optional, Dict


class VSCodeIntegration:
    """Handles VS Code integration and container attachment"""
    
    @staticmethod
    def check_vscode_installed() -> bool:
        """Check if VS Code is installed"""
        try:
            result = subprocess.run(["code", "--version"], capture_output=True, text=True)
            return result.returncode == 0
        except FileNotFoundError:
            return False
    
    @staticmethod
    def check_vscode_extensions() -> dict:
        """Check if required VS Code extensions are installed"""
        required_extensions = {
            "ms-vscode-remote.remote-containers": "Dev Containers",
            "ms-vscode-remote.vscode-remote-extensionpack": "Remote Development"
        }
        
        installed_extensions = {}
        
        try:
            result = subprocess.run(["code", "--list-extensions"], capture_output=True, text=True)
            if result.returncode == 0:
                installed = result.stdout.strip().split('\n')
                for ext_id, ext_name in required_extensions.items():
                    installed_extensions[ext_id] = {
                        "name": ext_name,
                        "installed": ext_id in installed
                    }
            return installed_extensions
        except FileNotFoundError:
            return {}
    
    @staticmethod
    def install_vscode_extensions(extensions: dict) -> bool:
        """Install missing VS Code extensions"""
        missing_extensions = [ext_id for ext_id, info in extensions.items() if not info["installed"]]
        
        if not missing_extensions:
            return True
        
        print("Installing missing VS Code extensions...")
        for ext_id in missing_extensions:
            try:
                print(f"Installing {extensions[ext_id]['name']}...")
                result = subprocess.run(["code", "--install-extension", ext_id], 
                                      capture_output=True, text=True)
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
    def get_container_info(compose_file: str, script_dir) -> Optional[dict]:
        """Get information about the running container"""
        try:
            print("🔍 Detecting running containers...")
            
            # Method 1: Use docker compose ps with specific format to get actual container IDs
            result = subprocess.run([
                "docker", "compose", "-f", compose_file, "ps", "--format", "table {{.Name}}\t{{.ID}}\t{{.Service}}\t{{.Status}}"
            ], capture_output=True, text=True, cwd=script_dir)
            
            if result.returncode == 0:
                print(f"📋 Docker compose output received, parsing containers...")
                lines = result.stdout.strip().split('\n')
                
                # Skip header line if present
                data_lines = [line for line in lines if not line.startswith('NAME') and line.strip()]
                
                for line in data_lines:
                    if line.strip():
                        parts = line.split('\t')
                        if len(parts) >= 4:
                            container_name = parts[0].strip()
                            container_id = parts[1].strip()
                            service_name = parts[2].strip()
                            status = parts[3].strip()
                            
                            print(f"   Found container: {container_name} (service: {service_name}, status: {status}, id: {container_id})")
                            
                            # Look for hydrus-related services
                            if any(keyword in service_name.lower() for keyword in ['hydrus', 'hydrus_cpu', 'hydrus_cuda']):
                                print(f"✅ Found Hydrus container: {container_name} with ID: {container_id}")
                                return {
                                    'name': container_name,
                                    'id': container_id,
                                    'status': status,
                                    'service': service_name
                                }
            
            # Method 2: Fallback to JSON format and extract from container list
            print("🔄 Fallback method 1: Using docker compose ps JSON format...")
            result = subprocess.run([
                "docker", "compose", "-f", compose_file, "ps", "--format", "json"
            ], capture_output=True, text=True, cwd=script_dir)
            
            if result.returncode == 0:
                for line in result.stdout.strip().split('\n'):
                    if line.strip():
                        try:
                            container = json.loads(line)
                            service_name = container.get('Service', '')
                            container_name = container.get('Name', '')
                            
                            # Look for hydrus-related services
                            if any(keyword in service_name.lower() for keyword in ['hydrus', 'hydrus_cpu', 'hydrus_cuda']):
                                # Get the actual container ID by querying docker directly with the container name
                                id_result = subprocess.run([
                                    "docker", "inspect", "--format", "{{.Id}}", container_name
                                ], capture_output=True, text=True)
                                
                                if id_result.returncode == 0:
                                    actual_container_id = id_result.stdout.strip()[:12]  # Use short ID
                                    state = container.get('State', '')
                                    
                                    print(f"✅ Found Hydrus container via JSON: {container_name} with actual ID: {actual_container_id}")
                                    return {
                                        'name': container_name,
                                        'id': actual_container_id,
                                        'status': state,
                                        'service': service_name
                                    }
                        except json.JSONDecodeError as e:
                            print(f"   ⚠️  JSON decode error for line: {line[:100]}...")
                            continue
            
            # Method 3: Direct docker ps fallback
            print("🔄 Fallback method 2: Using direct docker ps...")
            result = subprocess.run([
                "docker", "ps", "--format", "{{.Names}}\t{{.ID}}\t{{.Status}}\t{{.Image}}"
            ], capture_output=True, text=True)
            
            if result.returncode == 0:
                for line in result.stdout.strip().split('\n'):
                    if line.strip():
                        parts = line.split('\t')
                        if len(parts) >= 4:
                            name, container_id, status, image = parts[0], parts[1], parts[2], parts[3]
                            print(f"   Found container: {name} (image: {image}, id: {container_id})")
                            
                            # Look for hydrus in name or image
                            if any(keyword in name.lower() for keyword in ['hydrus', 'hydrus_cpu', 'hydrus_cuda']) or \
                               any(keyword in image.lower() for keyword in ['hydrus', 'docker-hydrus']):
                                print(f"✅ Found Hydrus container via direct docker ps: {name} with ID: {container_id}")
                                return {
                                    'name': name,
                                    'id': container_id,
                                    'status': status,
                                    'service': 'hydrus'
                                }
            
            print("❌ No Hydrus containers found")
            
        except Exception as e:
            print(f"❌ Error getting container info: {e}")
        
        return None
    
    @staticmethod
    def open_vscode_container(container_info: dict, parent_dir) -> bool:
        """Open VS Code and attach to the running container"""
        try:
            container_name = container_info['name']
            container_id = container_info['id']
            
            print(f"🚀 Opening VS Code and connecting to container:")
            print(f"   Name: {container_name}")
            print(f"   ID: {container_id}")
            print(f"   Service: {container_info.get('service', 'unknown')}")
            
            # Method 1: Try using the Dev Container extension with container ID
            print("📝 Attempting VS Code connection method 1 (Dev Container)...")
            try:
                result = subprocess.run([
                    "code", 
                    "--folder-uri", 
                    f"vscode-remote://attached-container+{container_id}/catkin_ws/src/hydrus-software-stack"
                ], capture_output=True, text=True, timeout=10)
                
                if result.returncode == 0:
                    print("✅ VS Code opened successfully using Dev Container method")
                    return True
                else:
                    print(f"⚠️  Dev Container method failed (exit code: {result.returncode})")
            except subprocess.TimeoutExpired:
                print("⚠️  Dev Container method timed out")
            except Exception as e:
                print(f"⚠️  Dev Container method error: {e}")
            
            # Method 2: Try opening the project directory
            print("📝 Attempting VS Code connection method 2 (Project directory)...")
            try:
                subprocess.Popen([
                    "code", 
                    str(parent_dir)
                ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                
                print("✅ VS Code opened with project directory")
                print(f"💡 To connect to the container manually:")
                print(f"   1. Press Ctrl+Shift+P")
                print(f"   2. Type 'Dev Containers: Attach to Running Container'")
                print(f"   3. Select container: {container_name}")
                print(f"   4. Open folder: /catkin_ws/src/hydrus-software-stack")
                return True
                
            except Exception as e:
                print(f"❌ Failed to open VS Code project directory: {e}")
            
            return False
                
        except Exception as e:
            print(f"❌ Failed to open VS Code: {e}")
            return False
    
    @staticmethod
    def setup_vscode_integration(compose_file: str, script_dir, parent_dir, auto_install_extensions: bool = False) -> bool:
        """Setup and launch VS Code integration if available"""
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
        missing_extensions = [info["name"] for info in extensions.values() if not info["installed"]]
        
        if missing_extensions:
            print(f"Missing required extensions: {', '.join(missing_extensions)}")
            
            if auto_install_extensions:
                print("Auto-installing missing extensions...")
                if not VSCodeIntegration.install_vscode_extensions(extensions):
                    print("Failed to install extensions. Skipping VS Code integration.")
                    return False
            else:
                print("Use --install-vscode-extensions to automatically install missing extensions.")
                print("Skipping VS Code integration without required extensions.")
                return False
        
        # Wait a moment for container to be fully ready
        print("Waiting for container to be ready...")
        time.sleep(3)
        
        # Get container information
        container_info = VSCodeIntegration.get_container_info(compose_file, script_dir)
        if not container_info:
            print("Could not find running Hydrus container. Skipping VS Code integration.")
            return False
        
        if 'running' not in container_info['status'].lower():
            print(f"Container is not running (status: {container_info['status']}). Skipping VS Code integration.")
            return False
        
        # Open VS Code
        return VSCodeIntegration.open_vscode_container(container_info, parent_dir)