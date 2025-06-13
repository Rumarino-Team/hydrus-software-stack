#!/usr/bin/env python3
"""
Hydrus Test Suite Runner
Replaces run_tests.sh with improved modularity and error handling
"""

import os
import sys
import time
import subprocess
import signal
from pathlib import Path
from typing import List, Dict, Optional


class HydrusTestRunner:
    def __init__(self):
        self.volume = os.environ.get("VOLUME", "false").lower() == "true"
        
        # Determine ROS directory based on VOLUME environment variable
        if self.volume:
            print("Using the Volume directory for building and testing the packages.")
            self.ros_dir = Path('/home/catkin_ws')
        else:
            print("Using the Copied Packages from Docker for building and testing.")
            self.ros_dir = Path('/catkin_ws')
        
        print(f"Using ROS workspace: {self.ros_dir}")
        
        # Test tracking
        self.total_tests = 0
        self.passed_tests = 0
        self.failed_tests = 0
        
        # Process tracking
        self.roscore_pid = None
        
    def _run_command(self, cmd: List[str], timeout: Optional[int] = None, 
                    check: bool = True, capture_output: bool = False,
                    env: Optional[Dict] = None, cwd: Optional[Path] = None) -> subprocess.CompletedProcess:
        """Run a command with proper error handling and timeout"""
        if env is None:
            env = os.environ.copy()
        
        try:
            return subprocess.run(cmd, timeout=timeout, check=check, 
                                capture_output=capture_output, env=env, cwd=cwd, text=True)
        except subprocess.TimeoutExpired:
            print(f"Command timed out after {timeout} seconds: {' '.join(cmd)}")
            raise
        except subprocess.CalledProcessError as e:
            if check:
                print(f"Command failed: {' '.join(cmd)}")
                print(f"Exit code: {e.returncode}")
                if capture_output and e.stderr:
                    print(f"Error: {e.stderr}")
            raise
    
    def _setup_ros_environment(self) -> Dict[str, str]:
        """Setup ROS environment"""
        env = os.environ.copy()
        env["ROS_DISTRO"] = "noetic"
        env["ROS_MASTER_URI"] = "http://localhost:11311"
        
        # Source ROS setup
        ros_setup_path = "/opt/ros/noetic/setup.bash"
        if Path(ros_setup_path).exists():
            # Add ROS paths to environment
            env["ROS_PACKAGE_PATH"] = f"/opt/ros/noetic/share"
            env["CMAKE_PREFIX_PATH"] = f"/opt/ros/noetic"
            env["LD_LIBRARY_PATH"] = f"/opt/ros/noetic/lib:{env.get('LD_LIBRARY_PATH', '')}"
            env["PATH"] = f"/opt/ros/noetic/bin:{env.get('PATH', '')}"
            env["PYTHONPATH"] = f"/opt/ros/noetic/lib/python3/dist-packages:{env.get('PYTHONPATH', '')}"
        
        return env
    
    def _build_workspace(self) -> Dict[str, str]:
        """Build the catkin workspace"""
        print("Building workspace...")
        env = self._setup_ros_environment()
        
        cmake_args = [
            "catkin_make",
            "--cmake-args",
            "-DCMAKE_BUILD_TYPE=Release",
            "-DPYTHON_EXECUTABLE=/usr/bin/python3",
            "-DPYTHON_INCLUDE_DIR=/usr/include/python3.9",
            "-DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.9.so"
        ]
        
        try:
            self._run_command(cmake_args, check=False, cwd=self.ros_dir, env=env)
        except subprocess.CalledProcessError:
            print("Build failed, but continuing...")
        
        # Source the workspace setup after building
        setup_file = self.ros_dir / "devel/setup.bash"
        if setup_file.exists():
            print("Workspace setup sourced successfully")
            # Update environment with workspace paths
            env["ROS_PACKAGE_PATH"] = f"{self.ros_dir}/src:{env.get('ROS_PACKAGE_PATH', '')}"
            env["CMAKE_PREFIX_PATH"] = f"{self.ros_dir}/devel:{env.get('CMAKE_PREFIX_PATH', '')}"
            env["LD_LIBRARY_PATH"] = f"{self.ros_dir}/devel/lib:{env.get('LD_LIBRARY_PATH', '')}"
            env["PYTHONPATH"] = f"{self.ros_dir}/devel/lib/python3/dist-packages:{env.get('PYTHONPATH', '')}"
        else:
            print("Warning: devel/setup.bash still not found after build")
        
        return env
    
    def _start_roscore(self, env: Dict[str, str]):
        """Start roscore in background"""
        print("Starting roscore...")
        
        try:
            process = subprocess.Popen(["roscore"], env=env, 
                                     stdout=subprocess.DEVNULL, 
                                     stderr=subprocess.DEVNULL)
            self.roscore_pid = process.pid
            time.sleep(5)
            
            # Check if roscore is running
            if not self._is_process_running(self.roscore_pid):
                print("Failed to start roscore")
                sys.exit(1)
            
            print("Roscore started successfully")
            
        except Exception as e:
            print(f"Failed to start roscore: {e}")
            sys.exit(1)
    
    def _is_process_running(self, pid: int) -> bool:
        """Check if a process is running"""
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False
    
    def _run_test(self, test_name: str, test_command: List[str], 
                  timeout: int = 300, env: Optional[Dict] = None):
        """Run a test and track results"""
        print("")
        print("----------------------------------------")
        print(f"Running: {test_name}")
        print("----------------------------------------")
        
        self.total_tests += 1
        
        if env is None:
            env = self._setup_ros_environment()
        
        try:
            # Source the workspace in the command environment
            bash_cmd = [
                "bash", "-c", 
                f"source {self.ros_dir}/devel/setup.bash 2>/dev/null || true; {' '.join(test_command)}"
            ]
            
            self._run_command(bash_cmd, timeout=timeout, env=env)
            print(f"✅ PASSED: {test_name}")
            self.passed_tests += 1
            
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            print(f"❌ FAILED: {test_name}")
            self.failed_tests += 1
    
    def _run_rostest(self, test_name: str, test_file: str, env: Dict[str, str]):
        """Run rostest with proper timeout and error handling"""
        print("")
        print("----------------------------------------")
        print(f"Running rostest: {test_name}")
        print("----------------------------------------")
        
        self.total_tests += 1
        
        try:
            bash_cmd = [
                "bash", "-c", 
                f"source {self.ros_dir}/devel/setup.bash 2>/dev/null || true; rostest autonomy {test_file}"
            ]
            
            self._run_command(bash_cmd, timeout=600, env=env)
            print(f"✅ PASSED: {test_name}")
            self.passed_tests += 1
            
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            print(f"❌ FAILED: {test_name}")
            self.failed_tests += 1
    
    def _run_unit_tests(self, env: Dict[str, str]):
        """Run unit tests that don't require ROS"""
        print("==========================================")
        print("Running Unit Tests (Non-ROS)")
        print("==========================================")
        
        # Tagging mission unit tests
        tagging_test_dir = self.ros_dir / "src/hydrus-software-stack/autonomy/src/mission_planner"
        if (tagging_test_dir / "tagging_mission_test.py").exists():
            self._run_test(
                "Tagging Mission Unit Tests",
                ["python3", "tagging_mission_test.py"],
                env=env
            )
    
    def _run_ros_integration_tests(self, env: Dict[str, str]):
        """Run ROS integration tests using rostest"""
        print("")
        print("==========================================")
        print("Running ROS Integration Tests (rostest)")
        print("==========================================")
        
        # Controller tests using rostest
        controller_test = self.ros_dir / "src/hydrus-software-stack/autonomy/test/controller.test"
        if controller_test.exists():
            self._run_rostest("Controller Tests", "controller.test", env)
        
        # Mission planner integration tests
        mission_planner_dir = self.ros_dir / "src/hydrus-software-stack/autonomy/src/mission_planner"
        
        # Set up PYTHONPATH for mission planner tests
        test_env = env.copy()
        pythonpath_additions = [
            str(self.ros_dir / "src"),
            str(self.ros_dir / "devel/lib/python3/dist-packages")
        ]
        test_env["PYTHONPATH"] = ":".join(pythonpath_additions + [test_env.get("PYTHONPATH", "")])
        
        # Slalom integration tests
        if (mission_planner_dir / "test_slalom_integration.py").exists():
            self._run_test(
                "Slalom Integration Tests",
                ["python3", "test_slalom_integration.py"],
                env=test_env
            )
        
        # Gate mission tests
        if (mission_planner_dir / "gate_mission_tester.py").exists():
            self._run_test(
                "Gate Mission Tests",
                ["python3", "gate_mission_tester.py"],
                env=test_env
            )
        
        # DVL driver tests if available
        dvl_test = self.ros_dir / "src/hydrus-software-stack/DVL/Wayfinder/driver_test.py"
        if dvl_test.exists():
            dvl_dir = self.ros_dir / "src/hydrus-software-stack/DVL/Wayfinder"
            self._run_test(
                "DVL Driver Tests",
                ["python3", "driver_test.py"],
                env=env
            )
    
    def _cleanup_roscore(self):
        """Clean up roscore process"""
        print("")
        print("Cleaning up...")
        if self.roscore_pid:
            try:
                os.kill(self.roscore_pid, signal.SIGTERM)
                time.sleep(2)
                # Force kill if still running
                if self._is_process_running(self.roscore_pid):
                    os.kill(self.roscore_pid, signal.SIGKILL)
            except OSError:
                pass  # Process already terminated
    
    def _print_summary(self):
        """Print final test summary"""
        print("")
        print("==========================================")
        print("TEST SUMMARY")
        print("==========================================")
        print(f"Total Tests: {self.total_tests}")
        print(f"Passed: {self.passed_tests}")
        print(f"Failed: {self.failed_tests}")
        
        if self.failed_tests == 0:
            print("🎉 ALL TESTS PASSED!")
            return 0
        else:
            print(f"💥 {self.failed_tests} TEST(S) FAILED!")
            return 1
    
    def _signal_handler(self, signum, frame):
        """Handle interrupt signals"""
        print("\nReceived interrupt signal, cleaning up...")
        self._cleanup_roscore()
        sys.exit(1)
    
    def main(self):
        """Main execution function"""
        print("==========================================")
        print("Running Hydrus Autonomy Test Suite")
        print("==========================================")
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        try:
            # Build workspace and setup environment
            env = self._build_workspace()
            
            # Start roscore for ROS tests
            self._start_roscore(env)
            
            # Run unit tests
            self._run_unit_tests(env)
            
            # Run ROS integration tests
            self._run_ros_integration_tests(env)
            
        finally:
            # Always clean up
            self._cleanup_roscore()
        
        # Print final results and exit
        exit_code = self._print_summary()
        sys.exit(exit_code)


if __name__ == "__main__":
    runner = HydrusTestRunner()
    runner.main()