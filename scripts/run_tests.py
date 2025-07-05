#!/usr/bin/env python3
"""
Hydrus Test Suite Runner
Replaces run_tests.sh with improved modularity and error handling
"""

import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

from scripts.scripts_utils import get_building_path


class HydrusTestRunner:
    def __init__(self):
        self.volume = os.environ.get("VOLUME", "false").lower() == "true"
        self.ros_dir = get_building_path(self.volume)

        print(f"Using ROS workspace: {self.ros_dir}")

        # Test tracking
        self.total_tests = 0
        self.passed_tests = 0
        self.failed_tests = 0

        # Process tracking
        self.roscore_pid = None

        # Logging setup
        self.test_logs_dir = self.ros_dir / "src/hydrus-software-stack/test_logs"
        self.test_logs_dir.mkdir(exist_ok=True)

        # Create timestamped log file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = self.test_logs_dir / f"test_run_{timestamp}.log"
        self.current_test_log = None

        print(f"Test logs will be saved to: {self.test_logs_dir}")
        print(f"Main log file: {self.log_file}")

    def _log_message(self, message: str, also_print: bool = True):
        """Log a message to the main log file and optionally print to console"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"

        # Write to main log file
        with open(self.log_file, "a") as f:
            f.write(log_entry)

        # Optionally print to console
        if also_print:
            print(message)

    def _run_command(
        self,
        cmd: List[str],
        timeout: Optional[int] = None,
        check: bool = True,
        capture_output: bool = False,
        env: Optional[Dict] = None,
        cwd: Optional[Path] = None,
    ) -> subprocess.CompletedProcess:
        """
        Execute a shell command with comprehensive error handling and logging.

        This method provides a robust wrapper around subprocess.run() with enhanced
        error reporting, timeout handling, and execution context logging. It's designed
        to execute commands in a controlled environment with clear feedback about
        what's happening and why failures occur.

        Args:
            cmd (List[str]): Command and arguments to execute as a list of strings.
                           Example: ['python3', '-m', 'pytest', 'tests/']
            timeout (Optional[int], optional): Maximum execution time in seconds.
                                             If None, no timeout is applied.
                                             Defaults to None.
            check (bool, optional): If True, raises CalledProcessError for non-zero
                                  exit codes. If False, allows commands to fail
                                  without raising exceptions. Defaults to True.
            capture_output (bool, optional): If True, captures stdout and stderr
                                           for programmatic access. If False,
                                           output goes directly to terminal.
                                           Defaults to False.
            env (Optional[Dict], optional): Environment variables for the command.
                                          If None, inherits current environment.
                                          Defaults to None.
            cwd (Optional[Path], optional): Working directory for command execution.
                                          If None, uses current working directory.
                                          Defaults to None.
        """
        if env is None:
            env = os.environ.copy()

        # Always print the command being executed
        cmd_str = " ".join(cmd)
        cwd_str = f" (in {cwd})" if cwd else ""
        print(f"🔧 Executing command: {cmd_str}{cwd_str}")

        try:
            return subprocess.run(
                cmd,
                timeout=timeout,
                check=check,
                capture_output=capture_output,
                env=env,
                cwd=cwd,
                text=True,
            )
        except FileNotFoundError as e:
            print(f"❌ Command not found: {cmd_str}")
            print(f"   File not found: {e.filename}")
            print(f"   Working directory: {cwd}")
            print(f"   Make sure the required tool is installed in the container")
            raise
        except subprocess.TimeoutExpired:
            print(f"⏰ Command timed out after {timeout} seconds: {cmd_str}")
            raise
        except subprocess.CalledProcessError as e:
            if check:
                print(f"❌ Command failed: {cmd_str}")
                print(f"   Exit code: {e.returncode}")
                if capture_output and e.stderr:
                    print(f"   Error: {e.stderr}")
            raise

    def _build_workspace(self):
        """Build the catkin workspace"""
        print("Building workspace...")
        env = os.environ.copy()

        # Build command that sources ROS environment first, then runs catkin_make
        cmake_command = f"source /opt/ros/noetic/setup.bash && catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.9 -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.9.so"

        cmake_args = ["bash", "-c", cmake_command]

        try:
            self._run_command(cmake_args, check=False, cwd=self.ros_dir, env=env)
        except subprocess.CalledProcessError:
            print("Build failed, but continuing...")

    def _start_roscore(self, env: Dict[str, str]):
        """Start roscore in background"""
        print("Starting roscore...")

        try:
            process = subprocess.Popen(
                ["roscore"],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
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

    def _run_rostest(self, test_name: str, test_file: str, env: Dict[str, str]):
        """Run rostest with proper timeout and error handling"""
        print("")
        print("----------------------------------------")
        print(f"Running rostest: {test_name}")
        print("----------------------------------------")

        self.total_tests += 1

        try:
            bash_cmd = [
                "bash",
                "-c",
                f"source {self.ros_dir}/devel/setup.bash 2>/dev/null || true; rostest autonomy {test_file}",
            ]

            self._run_command(bash_cmd, timeout=600, capture_output=False, env=env)
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

        # Define unit tests with their configuration
        unit_tests = [
            {
                "name": "Tagging Mission Unit Tests",
                "path": self.ros_dir
                / "src/hydrus-software-stack/autonomy/src/mission_planner/tagging_mission_test.py",
                "args": [],
                "timeout": 30,
                "infinite_loop": False,
                "description": "Unit tests for tagging mission functionality",
            }
        ]

        for test_config in unit_tests:
            if test_config["path"].exists():
                self._run_unified_test(
                    test_config["name"],
                    test_config["path"],
                    test_config["args"],
                    test_config["timeout"],
                    test_config["infinite_loop"],
                    test_config["description"],
                    env,
                )

    def _run_ros_integration_tests(self, env: Dict[str, str]):
        """Run ROS integration tests using rostest"""
        print("")
        print("==========================================")
        print("Running ROS Integration Tests (rostest)")
        print("==========================================")

        # Set up PYTHONPATH for mission planner tests
        test_env = env.copy()
        # Define ROS integration tests with their configuration
        ros_tests = [
            {
                "name": "Controller Tests",
                "path": "controller.test",  # Special case for rostest
                "args": [],
                "timeout": 600,
                "infinite_loop": False,
                "description": "ROS controller integration tests using rostest",
                "is_rostest": True,
            },
            {
                "name": "Slalom Integration Tests",
                "path": self.ros_dir
                / "src/hydrus-software-stack/autonomy/src/mission_planner/test_slalom_integration.py",
                "args": [],
                "timeout": 300,
                "infinite_loop": False,
                "description": "Slalom mission integration tests",
            },
            {
                "name": "Gate Mission Tests",
                "path": self.ros_dir
                / "src/hydrus-software-stack/autonomy/src/mission_planner/test_gate_mission.py",
                "args": [],
                "timeout": 300,
                "infinite_loop": False,
                "description": "Gate mission functionality tests",
            },
            {
                "name": "DVL Driver Tests",
                "path": self.ros_dir
                / "src/hydrus-software-stack/DVL/Wayfinder/driver_test.py",
                "args": [],
                "timeout": 60,
                "infinite_loop": False,
                "description": "DVL driver functionality tests",
            },
        ]

        for test_config in ros_tests:
            if test_config.get("is_rostest"):
                # Handle rostest differently
                self._run_rostest(test_config["name"], test_config["path"], test_env)
            elif test_config["path"].exists():
                self._run_unified_test(
                    test_config["name"],
                    test_config["path"],
                    test_config["args"],
                    test_config["timeout"],
                    test_config["infinite_loop"],
                    test_config["description"],
                    test_env,
                )

    def _run_script_tests(self, env: Dict[str, str]):
        """This tests is mainly to check that the scripts do not have obvious runtime errors.
        Like depedencies not being met, or syntax errors.
        It does not test the functionality of the scripts, just that they can run without crashing.
        """
        print("")
        print("==========================================")
        print("Running Script Tests")
        print("==========================================")

        # Define scripts to test with their configuration
        scripts_to_test = [
            {
                "name": "API Server",
                "path": self.ros_dir
                / "src/hydrus-software-stack/autonomy/src/api_server.py",
                "args": [],
                "timeout": 1.5,
                "infinite_loop": True,
                "description": "API Server script that runs indefinitely to handle API requests.",
            },
            {
                "name": "Controllers Node",
                "path": self.ros_dir
                / "src/hydrus-software-stack/autonomy/src/controllers.py",
                "args": [],
                "timeout": 1.5,
                "infinite_loop": True,
                "description": "Controllers Node script that runs indefinitely. Handles controller logic for the robot.",
            },
            {
                "name": "Computer Vision Node",
                "path": self.ros_dir
                / "src/hydrus-software-stack/autonomy/src/cv_publishers.py",
                "args": [],
                "timeout": 1.5,
                "infinite_loop": True,
                "description": "Computer vision Node script that runs indefinitely for image processing.",
            },
            {
                "name": "Controller Monitor Node",
                "path": self.ros_dir
                / "src/hydrus-software-stack/autonomy/scripts/controller/controller_monitor.py",
                "args": [],
                "timeout": 1.5,
                "infinite_loop": True,
                "description": "Controller Monitor Node script that runs indefinitely to manage missions.",
            },
        ]

        for script_config in scripts_to_test:
            self._run_unified_test(
                script_config["name"],
                script_config["path"],
                script_config["args"],
                script_config["timeout"],
                script_config["infinite_loop"],
                script_config["description"],
                env,
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

    def _run_unified_test(
        self,
        test_name: str,
        test_path: Path,
        test_args: List[str],
        timeout: int,
        is_infinite_loop: bool,
        description: str,
        env: Dict[str, str],
        working_dir: Optional[Path] = None,
    ):
        """Unified test execution function that handles all types of tests"""
        # Create individual test log file
        safe_test_name = "".join(
            c for c in test_name if c.isalnum() or c in (" ", "-", "_")
        ).replace(" ", "_")
        test_log_file = self.test_logs_dir / f"{safe_test_name}.log"

        # Log test start
        self._log_message(f"=== Starting Test: {test_name} ===")
        self._log_message(f"Description: {description}")
        self._log_message(f"Path: {test_path}")
        self._log_message(
            f"Expected behavior: {'Infinite loop' if is_infinite_loop else 'Clean exit'} (timeout: {timeout}s)"
        )

        print("")
        print("----------------------------------------")
        print(f"Running: {test_name}")
        print(f"Description: {description}")
        print(f"Path: {test_path}")
        if is_infinite_loop:
            print(f"Expected behavior: Infinite loop ({timeout}s timeout)")
        else:
            print(f"Expected behavior: Should exit cleanly (max {timeout}s)")
        print("----------------------------------------")

        self.total_tests += 1

        # Check if test file exists
        if not test_path.exists():
            error_msg = f"❌ FAILED: {test_name} - Test file not found at {test_path}"
            print(error_msg)
            self._log_message(error_msg)

            # Save individual test log
            with open(test_log_file, "w") as f:
                f.write(f"Test: {test_name}\n")
                f.write(f"Status: FAILED\n")
                f.write(f"Reason: Test file not found\n")
                f.write(f"Path: {test_path}\n")

            self.failed_tests += 1
            return

        # Prepare command with ROS environment sourcing
        test_command = ["python3", str(test_path)] + test_args
        bash_cmd = [
            "bash",
            "-c",
            f"source {self.ros_dir}/devel/setup.bash 2>/dev/null || true; {' '.join(test_command)}",
        ]
        cmd_str = " ".join(bash_cmd)

        # Set working directory
        cwd = working_dir or test_path.parent

        # Log command being executed
        self._log_message(f"Executing command: {cmd_str}")
        self._log_message(f"Working directory: {cwd}")

        try:
            if is_infinite_loop:
                # For infinite loop tests, run with timeout and capture output
                print(f"Running {test_name} with {timeout}-second timeout...")
                self._log_message(
                    f"Running {test_name} with {timeout}-second timeout..."
                )

                try:
                    result = self._run_command(
                        bash_cmd,
                        timeout=timeout,
                        check=False,
                        capture_output=True,
                        env=env,
                        cwd=cwd,
                    )

                    # If process finished within timeout, check exit code
                    if result.returncode != 0:
                        error_msg = f"❌ FAILED: {test_name} - Test exited with code {result.returncode} within {timeout} seconds"
                        print(error_msg)
                        self._log_message(error_msg)

                        if result.stderr:
                            print(f"Error output: {result.stderr}")
                            self._log_message(f"Error output: {result.stderr}")

                        # Save individual test log
                        with open(test_log_file, "w") as f:
                            f.write(f"Test: {test_name}\n")
                            f.write(f"Status: FAILED\n")
                            f.write(f"Exit Code: {result.returncode}\n")
                            f.write(f"Command: {cmd_str}\n")
                            f.write(f"Timeout: {timeout}s\n")
                            f.write(f"\n--- STDOUT ---\n{result.stdout}\n")
                            f.write(f"\n--- STDERR ---\n{result.stderr}\n")

                        self.failed_tests += 1
                    else:
                        warning_msg = f"⚠️  WARNING: {test_name} - Test completed within {timeout} seconds (expected infinite loop)"
                        print(warning_msg)
                        self._log_message(warning_msg)

                        if result.stdout:
                            print(f"Output: {result.stdout}")
                            self._log_message(f"Output: {result.stdout}")

                        # Save individual test log
                        with open(test_log_file, "w") as f:
                            f.write(f"Test: {test_name}\n")
                            f.write(f"Status: PASSED (WARNING)\n")
                            f.write(
                                f"Note: Completed within timeout (expected infinite loop)\n"
                            )
                            f.write(f"Command: {cmd_str}\n")
                            f.write(f"Timeout: {timeout}s\n")
                            f.write(f"\n--- STDOUT ---\n{result.stdout}\n")
                            f.write(f"\n--- STDERR ---\n{result.stderr}\n")

                        self.passed_tests += 1

                except subprocess.TimeoutExpired:
                    # This is expected for infinite loop tests
                    success_msg = f"✅ PASSED: {test_name} - Test ran successfully for {timeout} seconds"
                    print(success_msg)
                    self._log_message(success_msg)

                    # For timeout case, we don't have stdout/stderr from the result
                    # Save individual test log
                    with open(test_log_file, "w") as f:
                        f.write(f"Test: {test_name}\n")
                        f.write(f"Status: PASSED\n")
                        f.write(f"Note: Ran for expected timeout duration\n")
                        f.write(f"Command: {cmd_str}\n")
                        f.write(f"Timeout: {timeout}s\n")
                        f.write(
                            f"\n--- NOTE ---\nTest timed out as expected for infinite loop script\n"
                        )

                    self.passed_tests += 1

            else:
                # For finite tests, expect them to exit cleanly
                print(f"Running {test_name} (expecting clean exit)...")
                self._log_message(f"Running {test_name} (expecting clean exit)...")

                try:
                    result = self._run_command(
                        bash_cmd,
                        timeout=timeout,
                        check=False,
                        capture_output=True,
                        env=env,
                        cwd=cwd,
                    )

                    if result.returncode == 0:
                        success_msg = (
                            f"✅ PASSED: {test_name} - Test completed successfully"
                        )
                        print(success_msg)
                        self._log_message(success_msg)

                        if result.stdout:
                            print(f"Output: {result.stdout}")
                            self._log_message(f"Output: {result.stdout}")

                        # Save individual test log
                        with open(test_log_file, "w") as f:
                            f.write(f"Test: {test_name}\n")
                            f.write(f"Status: PASSED\n")
                            f.write(f"Exit Code: 0\n")
                            f.write(f"Command: {cmd_str}\n")
                            f.write(f"Timeout: {timeout}s\n")
                            f.write(f"\n--- STDOUT ---\n{result.stdout}\n")
                            f.write(f"\n--- STDERR ---\n{result.stderr}\n")

                        self.passed_tests += 1
                    else:
                        error_msg = f"❌ FAILED: {test_name} - Test failed with exit code {result.returncode}"
                        print(error_msg)
                        self._log_message(error_msg)

                        if result.stderr:
                            print(f"Error output: {result.stderr}")
                            self._log_message(f"Error output: {result.stderr}")
                        if result.stdout:
                            print(f"Standard output: {result.stdout}")
                            self._log_message(f"Standard output: {result.stdout}")

                        # Save individual test log
                        with open(test_log_file, "w") as f:
                            f.write(f"Test: {test_name}\n")
                            f.write(f"Status: FAILED\n")
                            f.write(f"Exit Code: {result.returncode}\n")
                            f.write(f"Command: {cmd_str}\n")
                            f.write(f"Timeout: {timeout}s\n")
                            f.write(f"\n--- STDOUT ---\n{result.stdout}\n")
                            f.write(f"\n--- STDERR ---\n{result.stderr}\n")

                        self.failed_tests += 1

                except subprocess.TimeoutExpired:
                    error_msg = f"❌ FAILED: {test_name} - Test timed out (exceeded {timeout} seconds)"
                    print(error_msg)
                    self._log_message(error_msg)

                    # Save individual test log
                    with open(test_log_file, "w") as f:
                        f.write(f"Test: {test_name}\n")
                        f.write(f"Status: FAILED\n")
                        f.write(f"Reason: Timeout after {timeout} seconds\n")
                        f.write(f"Command: {cmd_str}\n")

                    self.failed_tests += 1

        except FileNotFoundError:
            error_msg = (
                f"❌ FAILED: {test_name} - Python interpreter or test file not found"
            )
            print(error_msg)
            self._log_message(error_msg)

            # Save individual test log
            with open(test_log_file, "w") as f:
                f.write(f"Test: {test_name}\n")
                f.write(f"Status: FAILED\n")
                f.write(f"Reason: Python interpreter or test file not found\n")
                f.write(f"Command: {cmd_str}\n")

            self.failed_tests += 1
        except Exception as e:
            error_msg = f"❌ FAILED: {test_name} - Unexpected error: {e}"
            print(error_msg)
            self._log_message(error_msg)

            # Save individual test log
            with open(test_log_file, "w") as f:
                f.write(f"Test: {test_name}\n")
                f.write(f"Status: FAILED\n")
                f.write(f"Reason: Unexpected error\n")
                f.write(f"Error: {str(e)}\n")
                f.write(f"Command: {cmd_str}\n")

            self.failed_tests += 1

        # Log test completion
        self._log_message(f"=== Completed Test: {test_name} ===\n")

    def main(self):
        """Main execution function"""
        print("==========================================")
        print("Running Hydrus Autonomy Test Suite")
        print("==========================================")

        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        env = os.environ.copy()

        try:
            # Build workspace and setup environment
            self._build_workspace()

            # Start roscore for ROS tests
            # self._start_roscore(env)

            # Run unit tests
            self._run_unit_tests(env)

            # Run ROS integration tests
            self._run_ros_integration_tests(env)

            # Run script tests
            self._run_script_tests(env)

        finally:
            # Always clean up
            self._cleanup_roscore()

        # Print final results and exit
        exit_code = self._print_summary()
        sys.exit(exit_code)


if __name__ == "__main__":
    runner = HydrusTestRunner()
    runner.main()
