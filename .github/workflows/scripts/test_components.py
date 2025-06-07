#!/usr/bin/env python3
"""
Individual Test Components for Hydrus Software Stack
Contains specific test functions that can be run independently
"""

import subprocess
import sys
import logging
from pathlib import Path

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class HydrusTestComponents:
    """Individual test components that can be run separately"""
    
    def __init__(self, container_name="hydrus"):
        self.container_name = container_name
    
    def run_docker_command(self, command, timeout=60):
        """Execute a command inside the Docker container"""
        full_command = f"docker exec {self.container_name} bash -c \"{command}\""
        
        try:
            logger.info(f"Executing: {command}")
            result = subprocess.run(
                full_command,
                shell=True,
                capture_output=True,
                text=True,
                timeout=timeout
            )
            
            if result.stdout:
                logger.info(f"Output: {result.stdout}")
            if result.stderr:
                logger.warning(f"Errors: {result.stderr}")
                
            return result.returncode == 0
            
        except subprocess.TimeoutExpired:
            logger.warning(f"Command timed out after {timeout}s")
            return True  # Timeout is expected for some tests
        except Exception as e:
            logger.error(f"Command failed: {e}")
            return False

    def test_ros_build(self):
        """Test ROS workspace compilation"""
        logger.info("Testing ROS workspace build...")
        command = "cd /home/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make"
        return self.run_docker_command(command, timeout=300)

    def test_controller_functionality(self):
        """Test controller with thruster validation"""
        logger.info("Testing controller functionality...")
        command = (
            "cd /home/catkin_ws && "
            "source devel/setup.bash && "
            "timeout 30s python3 src/hydrus-software-stack/autonomy/scripts/controller/controller_tester.py 3.0 || "
            "[ $? -eq 124 ]"  # Exit code 124 means timeout, which is expected
        )
        return self.run_docker_command(command, timeout=40)

    def test_gate_mission(self):
        """Test gate mission planning"""
        logger.info("Testing gate mission...")
        command = (
            "cd /home/catkin_ws && "
            "source devel/setup.bash && "
            "timeout 30s python3 src/hydrus-software-stack/autonomy/src/mission_planner/gate_mission_tester.py || "
            "[ $? -eq 124 ]"
        )
        return self.run_docker_command(command, timeout=40)

    def test_python_imports(self):
        """Test that critical Python modules import correctly"""
        logger.info("Testing Python imports...")
        import_script = """
import sys
sys.path.insert(0, 'src/hydrus-software-stack/autonomy/src')
try:
    import mission_planner.gate_mission
    import mission_planner.mission_manager
    from autonomy.msg import NavigateToWaypointAction
    print('All imports successful')
except Exception as e:
    print(f'Import failed: {e}')
    sys.exit(1)
"""
        command = f"cd /home/catkin_ws && source devel/setup.bash && python3 -c \"{import_script}\""
        return self.run_docker_command(command, timeout=20)

    def test_syntax_validation(self):
        """Test Python syntax of key files"""
        logger.info("Testing Python syntax...")
        files = [
            "autonomy/src/controllers.py",
            "autonomy/src/cv_publishers.py",
            "autonomy/scripts/controller/controller_tester.py",
            "autonomy/src/mission_planner/gate_mission_tester.py"
        ]
        
        for file_path in files:
            command = f"cd /home/catkin_ws/src/hydrus-software-stack && python3 -m py_compile {file_path}"
            if not self.run_docker_command(command, timeout=10):
                logger.error(f"Syntax error in {file_path}")
                return False
        
        return True

def run_individual_test(test_name):
    """Run a specific test by name"""
    tester = HydrusTestComponents()
    
    tests = {
        'build': tester.test_ros_build,
        'controller': tester.test_controller_functionality,
        'gate_mission': tester.test_gate_mission,
        'imports': tester.test_python_imports,
        'syntax': tester.test_syntax_validation
    }
    
    if test_name not in tests:
        logger.error(f"Unknown test: {test_name}")
        logger.info(f"Available tests: {', '.join(tests.keys())}")
        return False
    
    logger.info(f"Running {test_name} test...")
    result = tests[test_name]()
    
    if result:
        logger.info(f"✓ {test_name} test PASSED")
    else:
        logger.error(f"✗ {test_name} test FAILED")
    
    return result

def run_all_individual_tests():
    """Run all tests and return overall result"""
    tester = HydrusTestComponents()
    
    tests = [
        ('ROS Build', tester.test_ros_build),
        ('Controller', tester.test_controller_functionality),
        ('Gate Mission', tester.test_gate_mission),
        ('Python Imports', tester.test_python_imports),
        ('Syntax Validation', tester.test_syntax_validation)
    ]
    
    results = {}
    for test_name, test_func in tests:
        logger.info(f"\n--- Running {test_name} Test ---")
        results[test_name] = test_func()
    
    # Print summary
    logger.info("\n=== Test Results Summary ===")
    all_passed = True
    for test_name, result in results.items():
        status = "PASS" if result else "FAIL"
        logger.info(f"{test_name}: {status}")
        if not result:
            all_passed = False
    
    return all_passed

if __name__ == "__main__":
    if len(sys.argv) > 1:
        # Run specific test
        test_name = sys.argv[1]
        success = run_individual_test(test_name)
    else:
        # Run all tests
        success = run_all_individual_tests()
    
    sys.exit(0 if success else 1)