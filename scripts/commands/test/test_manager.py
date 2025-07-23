import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

from ..utils import get_building_path
from .test_cases import TestCase
from .test_logger import TestLogManager, run_command_with_logging


class HydrusTestManager:
    def __init__(self, volume: bool = False, debug_mode: bool = False):
        self.volume = volume
        self.debug_mode = debug_mode
        self.ros_dir = get_building_path(self.volume)
        # Test tracking
        self.total_tests = 0
        self.passed_tests = 0
        self.failed_tests = 0
        # Process tracking
        self.roscore_pid: Optional[int] = None

        # Initialize test logging system
        self.log_manager = TestLogManager()

        # Legacy logging setup for compatibility
        self.test_logs_dir = self.ros_dir / "src/hydrus-software-stack/test_logs"
        self.test_logs_dir.mkdir(exist_ok=True)

        # Create timestamped log file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = self.test_logs_dir / f"test_run_{timestamp}.log"

    def _run_command(
        self,
        cmd: List[str],
        timeout: Optional[int] = None,
        check: bool = True,
        capture_output: bool = False,
        env: Optional[Dict] = None,
        cwd: Optional[Path] = None,
    ) -> subprocess.CompletedProcess:
        """Run a command with optional timeout and error handling"""
        if self.debug_mode:
            print(f"DEBUG: Running command: {' '.join(cmd)}")

        try:
            result = subprocess.run(
                cmd,
                timeout=timeout,
                check=check,
                capture_output=capture_output,
                text=True,
                env=env,
                cwd=cwd,
            )
            return result
        except subprocess.TimeoutExpired as e:
            print(f"⏰ Command timed out after {timeout} seconds: {' '.join(cmd)}")
            if self.debug_mode:
                print(f"DEBUG: Timeout details: {e}")
            raise
        except subprocess.CalledProcessError as e:
            if self.debug_mode:
                print(f"DEBUG: Command failed with exit code {e.returncode}")
                if e.stdout:
                    print(f"DEBUG: stdout: {e.stdout}")
                if e.stderr:
                    print(f"DEBUG: stderr: {e.stderr}")
            raise

    def _build_workspace(self):
        """Build the catkin workspace"""
        print("\n🔨 Building catkin workspace...")
        try:
            bash_cmd = [
                "bash",
                "-c",
                f"source /opt/ros/noetic/setup.bash && cd {self.ros_dir} && catkin_make",
            ]
            self._run_command(bash_cmd, timeout=300)
            print("✅ Workspace built successfully")
        except subprocess.CalledProcessError:
            print("❌ Failed to build workspace")
            sys.exit(1)
        except subprocess.TimeoutExpired:
            print("❌ Workspace build timed out")
            sys.exit(1)

    def _is_process_running(self, pid: int) -> bool:
        """Check if a process is running"""
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False

    def _signal_handler(self, signum, frame):
        """Handle interrupt signals gracefully"""
        print("\n🛑 Received interrupt signal. Cleaning up...")
        self._cleanup_roscore()
        sys.exit(1)

    def _start_roscore(self, env: Dict[str, str]):
        """Start roscore in background"""
        print("🤖 Starting roscore...")
        try:
            # Source ROS environment and start roscore
            bash_cmd = [
                "bash",
                "-c",
                f"source /opt/ros/noetic/setup.bash && source {self.ros_dir}/devel/setup.bash && roscore",
            ]

            roscore_process = subprocess.Popen(
                bash_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env
            )
            self.roscore_pid = roscore_process.pid

            # Give roscore time to start
            time.sleep(3)

            if self._is_process_running(self.roscore_pid):
                print(f"✅ roscore started (PID: {self.roscore_pid})")
            else:
                print("❌ Failed to start roscore")
                self.roscore_pid = None

        except Exception as e:
            print(f"❌ Error starting roscore: {e}")
            self.roscore_pid = None

    def _cleanup_roscore(self):
        """Clean up roscore process"""
        if self.roscore_pid and self._is_process_running(self.roscore_pid):
            print(f"🧹 Cleaning up roscore (PID: {self.roscore_pid})")
            try:
                os.kill(self.roscore_pid, signal.SIGTERM)
                time.sleep(2)
                if self._is_process_running(self.roscore_pid):
                    os.kill(self.roscore_pid, signal.SIGKILL)
                print("✅ roscore cleaned up")
            except OSError:
                print("⚠️  roscore process already terminated")
            finally:
                self.roscore_pid = None

    def _run_rostest(self, test_name: str, test_file: str, env: Dict[str, str]):
        """Run rostest with proper timeout and error handling"""
        print("")
        print("----------------------------------------")
        print(f"Running rostest: {test_name}")
        print("----------------------------------------")

        self.total_tests += 1

        # Use the new logging system
        with self.log_manager.capture_test(test_name) as test_capture:
            test_capture.log_info(f"Test file: {test_file}")
            test_capture.log_info("Test type: rostest")

            try:
                bash_cmd = [
                    "bash",
                    "-c",
                    f"source /opt/ros/noetic/setup.bash && source {self.ros_dir}/devel/setup.bash && rostest autonomy {test_file}",
                ]

                cmd_str = " ".join(bash_cmd)
                test_capture.log_info(f"Command: {cmd_str}")

                # Use the new logging system with command execution
                return_code = run_command_with_logging(
                    bash_cmd, test_capture, timeout=600, cwd=self.ros_dir, env=env
                )

                if return_code == 0:
                    print(f"✅ PASSED: {test_name}")
                    self.passed_tests += 1
                else:
                    print(f"❌ FAILED: {test_name} (exit code: {return_code})")
                    self.failed_tests += 1

            except Exception as e:
                test_capture.log_error(f"Unexpected error: {e}")
                print(f"❌ FAILED: {test_name} (error: {e})")
                self.failed_tests += 1

    def _run_unified_test(
        self,
        test_name: str,
        test_path: Path,
        args: List[str],
        timeout: int,
        infinite_loop: bool,
        description: str,
        env: Dict[str, str],
    ):
        """Run a unified test using the new logging system"""
        print("")
        print("----------------------------------------")
        print(f"Running test: {test_name}")
        print(f"Description: {description}")
        print("----------------------------------------")

        self.total_tests += 1

        # Use the new logging system
        with self.log_manager.capture_test(test_name) as test_capture:
            test_capture.log_info(f"Test path: {test_path}")
            test_capture.log_info(f"Args: {args}")
            test_capture.log_info(f"Timeout: {timeout}s")
            test_capture.log_info(f"Infinite loop: {infinite_loop}")

            try:
                if test_path.suffix == ".py":
                    # Python test - need to source ROS environment for autonomy package
                    if any(
                        ros_keyword in str(test_path)
                        for ros_keyword in [
                            "autonomy",
                            "mission_planner",
                            "api_server",
                            "controllers",
                            "cv_publishers",
                        ]
                    ):
                        # Run with sourced ROS environment
                        cmd_script = f"source /opt/ros/noetic/setup.bash && source {self.ros_dir}/devel/setup.bash && python3 {test_path} {' '.join(args)}"
                        cmd = ["bash", "-c", cmd_script]
                    else:
                        # Regular Python test
                        cmd = ["python3", str(test_path)] + args
                else:
                    # Shell script or other executable
                    cmd = [str(test_path)] + args

                # Log the command
                cmd_str = " ".join(cmd)
                test_capture.log_info(f"Command: {cmd_str}")

                # Execute test with timeout unless it's an infinite loop test
                test_timeout = timeout if not infinite_loop else timeout

                # Use the new logging system with command execution
                return_code = run_command_with_logging(
                    cmd, test_capture, timeout=test_timeout, cwd=self.ros_dir, env=env
                )

                if return_code == 0:
                    print(f"✅ PASSED: {test_name}")
                    self.passed_tests += 1
                elif return_code == -1 and infinite_loop:
                    # Timeout is expected for infinite loop tests
                    print(f"✅ PASSED: {test_name} (timed out as expected)")
                    self.passed_tests += 1
                else:
                    print(f"❌ FAILED: {test_name} (exit code: {return_code})")
                    self.failed_tests += 1

            except FileNotFoundError:
                test_capture.log_error(f"Test file not found: {test_path}")
                print(f"❌ FAILED: {test_name} (test file not found: {test_path})")
                self.failed_tests += 1
            except Exception as e:
                test_capture.log_error(f"Unexpected error: {e}")
                print(f"❌ FAILED: {test_name} (error: {e})")
                self.failed_tests += 1

    def execute_test_cases(self, test_cases: List[TestCase], env: Dict[str, str]):
        """Execute a list of test cases using their strategies"""
        for test_case in test_cases:
            test_case.strategy.execute(test_case, env, self)

    def _print_summary(self) -> int:
        """Print test results summary, create log archive, and return exit code"""
        print("")
        print("==========================================")
        print("TEST RESULTS SUMMARY")
        print("==========================================")
        print(f"Total tests run: {self.total_tests}")
        print(f"✅ Passed: {self.passed_tests}")
        print(f"❌ Failed: {self.failed_tests}")

        # End the logging session
        self.log_manager.end_session()

        # Print detailed session report
        self.log_manager.print_session_report()

        # Create log archive
        try:
            archive_path = self.log_manager.create_archive()
            print(f"📦 Log archive created: {archive_path}")

            # For CI/CD integration, also create a fixed name archive
            ci_archive_path = Path.cwd() / "test_logs_latest.zip"
            ci_archive = self.log_manager.create_archive(ci_archive_path)
            print(f"📦 CI/CD archive: {ci_archive}")

        except Exception as e:
            print(f"⚠️  Warning: Failed to create log archive: {e}")

        # Legacy logging
        print(f"📁 Detailed logs available in: {self.log_manager.base_log_dir}")
        print(f"📁 Legacy logs available in: {self.test_logs_dir}")

        if self.failed_tests == 0:
            print("\n🎉 All tests passed!")
            return 0
        else:
            success_rate = (
                (self.passed_tests / self.total_tests) * 100
                if self.total_tests > 0
                else 0
            )
            print(f"\n📊 Success rate: {success_rate:.1f}%")
            return 1
