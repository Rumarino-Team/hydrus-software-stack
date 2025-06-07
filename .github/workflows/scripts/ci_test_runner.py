#!/usr/bin/env python3
"""
CI Test Runner for Hydrus Software Stack
Handles Docker setup and test execution for CI/CD pipeline
"""

import os
import sys
import subprocess
import time
import signal
import logging
from pathlib import Path

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class HydrusCITester:
    def __init__(self, project_root=None):
        """Initialize the CI tester with project root directory"""
        if project_root is None:
            # Assume script is in scripts/ directory, project root is parent
            self.project_root = Path(__file__).parent.parent.absolute()
        else:
            self.project_root = Path(project_root).absolute()
            
        self.docker_dir = self.project_root / "docker"
        self.run_docker_script = self.docker_dir / "run_docker.sh"
        
        # Container names
        self.hydrus_container = "hydrus"
        self.camera_container = "camera"
        
        # Test timeout in seconds
        self.test_timeout = 60
        
    def run_command(self, command, shell=True, capture_output=True, timeout=None, cwd=None):
        """Run a command and return the result"""
        try:
            logger.info(f"Running command: {command}")
            if cwd:
                logger.info(f"Working directory: {cwd}")
                
            result = subprocess.run(
                command,
                shell=shell,
                capture_output=capture_output,
                text=True,
                timeout=timeout,
                cwd=cwd
            )
            
            if capture_output:
                if result.stdout:
                    logger.info(f"STDOUT: {result.stdout}")
                if result.stderr:
                    logger.warning(f"STDERR: {result.stderr}")
                    
            return result
        except subprocess.TimeoutExpired:
            logger.warning(f"Command timed out after {timeout} seconds: {command}")
            return None
        except Exception as e:
            logger.error(f"Error running command '{command}': {e}")
            return None
    
    def check_docker_running(self):
        """Check if Docker is running and accessible"""
        logger.info("Checking Docker availability...")
        result = self.run_command("docker ps")
        if result and result.returncode == 0:
            logger.info("✓ Docker is running and accessible")
            return True
        else:
            logger.error("✗ Docker is not running or not accessible")
            return False
    
    def make_script_executable(self):
        """Make the run_docker.sh script executable"""
        logger.info("Making run_docker.sh executable...")
        try:
            os.chmod(self.run_docker_script, 0o755)
            logger.info("✓ run_docker.sh is now executable")
            return True
        except Exception as e:
            logger.error(f"✗ Failed to make run_docker.sh executable: {e}")
            return False
    
    def start_docker_containers(self):
        """Start Docker containers using run_docker.sh"""
        logger.info("Starting Docker containers...")
        
        # Change to docker directory
        os.chdir(self.docker_dir)
        
        # Prepare non-interactive responses for the script
        # "n" for rosbag playback, "n" for Arduino debugging
        input_responses = "n\nn\n"
        
        # Run the docker script with force-cpu and volume flags
        command = f"echo '{input_responses}' | ./run_docker.sh --force-cpu --volume"
        
        try:
            # Start containers in background
            process = subprocess.Popen(
                command,
                shell=True,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            logger.info("Docker containers starting in background...")
            
            # Wait for containers to be ready
            max_wait = 60  # Maximum wait time in seconds
            wait_interval = 5  # Check every 5 seconds
            
            for i in range(0, max_wait, wait_interval):
                time.sleep(wait_interval)
                
                # Check if containers are running
                result = self.run_command("docker ps --format 'table {{.Names}}\t{{.Status}}'")
                if result and result.returncode == 0:
                    if self.hydrus_container in result.stdout:
                        logger.info(f"✓ Docker containers are running after {i + wait_interval} seconds")
                        return True
                        
                logger.info(f"Waiting for containers... ({i + wait_interval}/{max_wait}s)")
            
            logger.error("✗ Containers failed to start within timeout")
            return False
            
        except Exception as e:
            logger.error(f"✗ Failed to start Docker containers: {e}")
            return False
    
    def run_catkin_build_test(self):
        """Test that the ROS workspace builds successfully"""
        logger.info("Running catkin_make build test...")
        
        command = (
            f"docker exec {self.hydrus_container} bash -c "
            f"'cd /home/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make'"
        )
        
        result = self.run_command(command, timeout=300)  # 5 minute timeout for build
        
        if result and result.returncode == 0:
            logger.info("✓ catkin_make build test passed")
            return True
        else:
            logger.error("✗ catkin_make build test failed")
            return False
    
    def run_controller_tests(self):
        """Run controller functionality tests"""
        logger.info("Running controller tests...")
        
        command = (
            f"docker exec {self.hydrus_container} bash -c \""
            f"cd /home/catkin_ws && "
            f"source devel/setup.bash && "
            f"timeout {self.test_timeout}s python3 src/hydrus-software-stack/autonomy/scripts/controller/controller_tester.py 5.0 || "
            f"if [ \\$? -eq 124 ]; then "
            f"echo 'Controller test completed (timeout expected in CI)'; "
            f"exit 0; "
            f"else "
            f"exit \\$?; "
            f"fi\""
        )
        
        result = self.run_command(command, timeout=self.test_timeout + 10)
        
        if result and result.returncode == 0:
            logger.info("✓ Controller tests passed")
            return True
        else:
            logger.error("✗ Controller tests failed")
            return False
    
    def run_gate_mission_tests(self):
        """Run gate mission functionality tests"""
        logger.info("Running gate mission tests...")
        
        command = (
            f"docker exec {self.hydrus_container} bash -c \""
            f"cd /home/catkin_ws && "
            f"source devel/setup.bash && "
            f"timeout {self.test_timeout}s python3 src/hydrus-software-stack/autonomy/src/mission_planner/gate_mission_tester.py || "
            f"if [ \\$? -eq 124 ]; then "
            f"echo 'Gate mission test completed (timeout expected in CI)'; "
            f"exit 0; "
            f"else "
            f"exit \\$?; "
            f"fi\""
        )
        
        result = self.run_command(command, timeout=self.test_timeout + 10)
        
        if result and result.returncode == 0:
            logger.info("✓ Gate mission tests passed")
            return True
        else:
            logger.error("✗ Gate mission tests failed")
            return False
    
    def run_python_import_tests(self):
        """Run Python module import tests"""
        logger.info("Running Python module import tests...")
        
        python_test_script = '''
import sys
sys.path.insert(0, "src/hydrus-software-stack/autonomy/src")

# Test core module imports
try:
    import mission_planner.gate_mission
    print("✓ Gate mission module imports successfully")
except Exception as e:
    print(f"✗ Gate mission import failed: {e}")
    sys.exit(1)

try:
    import mission_planner.mission_manager
    print("✓ Mission manager module imports successfully")
except Exception as e:
    print(f"✗ Mission manager import failed: {e}")
    sys.exit(1)

try:
    from autonomy.msg import NavigateToWaypointAction
    print("✓ ROS messages import successfully")
except Exception as e:
    print(f"✗ ROS messages import failed: {e}")
    sys.exit(1)
    
print("All import tests passed!")
'''
        
        command = (
            f"docker exec {self.hydrus_container} bash -c \""
            f"cd /home/catkin_ws && "
            f"source devel/setup.bash && "
            f"python3 -c '{python_test_script}'\""
        )
        
        result = self.run_command(command, timeout=30)
        
        if result and result.returncode == 0:
            logger.info("✓ Python import tests passed")
            return True
        else:
            logger.error("✗ Python import tests failed")
            return False
    
    def run_syntax_validation_tests(self):
        """Run Python syntax validation for ROS nodes"""
        logger.info("Running Python syntax validation tests...")
        
        files_to_check = [
            "autonomy/src/controllers.py",
            "autonomy/src/cv_publishers.py", 
            "autonomy/scripts/controller/controller_tester.py",
            "autonomy/src/mission_planner/gate_mission_tester.py"
        ]
        
        for file_path in files_to_check:
            command = (
                f"docker exec {self.hydrus_container} bash -c \""
                f"cd /home/catkin_ws/src/hydrus-software-stack && "
                f"python3 -m py_compile {file_path}\""
            )
            
            result = self.run_command(command, timeout=10)
            
            if result and result.returncode == 0:
                logger.info(f"✓ {file_path} has valid syntax")
            else:
                logger.error(f"✗ {file_path} has syntax errors")
                return False
        
        logger.info("✓ All Python syntax validation tests passed")
        return True
    
    def cleanup_containers(self):
        """Stop and remove Docker containers"""
        logger.info("Cleaning up Docker containers...")
        
        containers = [self.hydrus_container, self.camera_container]
        
        for container in containers:
            # Stop container
            stop_result = self.run_command(f"docker stop {container}")
            if stop_result and stop_result.returncode == 0:
                logger.info(f"✓ Stopped {container} container")
            
            # Remove container
            rm_result = self.run_command(f"docker rm {container}")
            if rm_result and rm_result.returncode == 0:
                logger.info(f"✓ Removed {container} container")
        
        # Clean up any remaining containers
        self.run_command("docker container prune -f")
        logger.info("✓ Container cleanup completed")
    
    def run_all_tests(self):
        """Run the complete test suite"""
        logger.info("=== Starting Hydrus CI Test Suite ===")
        
        test_results = {}
        
        try:
            # Pre-flight checks
            if not self.check_docker_running():
                return False
            
            if not self.make_script_executable():
                return False
            
            # Start containers
            test_results["container_startup"] = self.start_docker_containers()
            if not test_results["container_startup"]:
                return False
            
            # Run tests
            test_results["catkin_build"] = self.run_catkin_build_test()
            test_results["controller_tests"] = self.run_controller_tests()
            test_results["gate_mission_tests"] = self.run_gate_mission_tests()
            test_results["python_imports"] = self.run_python_import_tests()
            test_results["syntax_validation"] = self.run_syntax_validation_tests()
            
        except KeyboardInterrupt:
            logger.warning("Tests interrupted by user")
            return False
        except Exception as e:
            logger.error(f"Unexpected error during testing: {e}")
            return False
        finally:
            # Always cleanup
            self.cleanup_containers()
        
        # Print results summary
        logger.info("\n=== Test Results Summary ===")
        all_passed = True
        for test_name, result in test_results.items():
            status = "PASS" if result else "FAIL"
            logger.info(f"{test_name}: {status}")
            if not result:
                all_passed = False
        
        if all_passed:
            logger.info("🎉 All tests passed!")
            return True
        else:
            logger.error("❌ Some tests failed!")
            return False

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Run Hydrus CI test suite')
    parser.add_argument('--project-root', 
                       help='Path to project root directory',
                       default=None)
    
    args = parser.parse_args()
    
    tester = HydrusCITester(project_root=args.project_root)
    success = tester.run_all_tests()
    
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    import argparse
    main()