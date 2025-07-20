import os
import signal
import subprocess
import sys
import time
from abc import ABC, abstractmethod
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import typer
import yaml

from ..utils import get_building_path
from .test_config_loader import (
    DistributedTestConfigLoader,
    TestConfig,
    get_strategy_class,
)
from .test_logger import TestLogManager, run_command_with_logging

# Create the test app
test_app = typer.Typer()

# Global state to store volume setting
_volume_mode = False


# === Strategy Pattern for Test Execution ===


class TestExecutionStrategy(ABC):
    @abstractmethod
    def execute(self, test_case: "TestCase", env: Dict, runner: "HydrusTestManager"):
        pass


class UnitTestStrategy(TestExecutionStrategy):
    def execute(self, test_case, env, runner):
        runner._run_unified_test(
            test_case.name,
            test_case.path,
            test_case.args,
            test_case.timeout,
            test_case.infinite_loop,
            test_case.description,
            env,
        )


class RosTestStrategy(TestExecutionStrategy):
    def execute(self, test_case, env, runner):
        runner._run_rostest(test_case.name, str(test_case.path), env)


class ScriptTestStrategy(TestExecutionStrategy):
    def execute(self, test_case, env, runner):
        runner._run_unified_test(
            test_case.name,
            test_case.path,
            test_case.args,
            test_case.timeout,
            test_case.infinite_loop,
            test_case.description,
            env,
        )


# === TestCase Definition ===


class TestCase:
    def __init__(
        self,
        name: str,
        path: Path,
        args: List[str],
        timeout: int,
        infinite_loop: bool,
        description: str,
        strategy: TestExecutionStrategy,
    ):
        self.name = name
        self.path = path
        self.args = args
        self.timeout = timeout
        self.infinite_loop = infinite_loop
        self.description = description
        self.strategy = strategy


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


def _is_directory_target(target: str, repo_root: Path) -> bool:
    """Determine if the target is a directory path."""
    # Check if it's a directory path
    if target in [".", "./", ""]:
        return True

    # Check if it's an absolute directory path
    if target.startswith("/"):
        abs_path = Path(target)
        return abs_path.exists() and abs_path.is_dir()

    # Check if it's a relative directory path from repo root
    rel_path = repo_root / target
    return rel_path.exists() and rel_path.is_dir()


def _run_tests_in_directory(
    target: str,
    repo_root: Path,
    config_loader: DistributedTestConfigLoader,
    test_manager: HydrusTestManager,
    env: Dict[str, str],
):
    """Run all tests found in the specified directory."""
    # Determine search directory
    if target == "." or target == "./":
        search_dir = Path.cwd()
        try:
            search_dir.relative_to(repo_root)
        except ValueError:
            search_dir = repo_root
    elif target.startswith("/"):
        search_dir = Path(target)
        if not str(search_dir).startswith(str(repo_root)):
            typer.echo(f"❌ Directory must be within repository: {repo_root}")
            raise typer.Exit(1)
    else:
        search_dir = repo_root / target

    if not search_dir.exists():
        typer.echo(f"❌ Directory not found: {search_dir}")
        raise typer.Exit(1)

    typer.echo(f"🔍 Running tests in directory: {search_dir}")

    # Find all .hss files in the directory and subdirectories
    hss_files = list(search_dir.rglob("*.hss"))

    if not hss_files:
        typer.echo(f"❌ No .hss files found in {search_dir}")
        return

    typer.echo(f"📁 Found {len(hss_files)} .hss files")

    # Collect all test cases from the directory
    all_test_cases = []

    for hss_file in hss_files:
        try:
            with open(hss_file, "r") as f:
                hss_content = yaml.safe_load(f)

            if not hss_content or "tests" not in hss_content:
                continue

            # Convert tests to TestCase objects
            for test_config_data in hss_content["tests"]:
                if test_config_data.get("enabled", True):  # Only run enabled tests
                    test_config = TestConfig(
                        name=test_config_data.get("name", "Unnamed Test"),
                        path=test_config_data.get("path", ""),
                        args=test_config_data.get("args", []),
                        timeout=test_config_data.get("timeout", 300),
                        infinite_loop=test_config_data.get("infinite_loop", False),
                        description=test_config_data.get("description", ""),
                        strategy=test_config_data.get("strategy", "UnitTestStrategy"),
                        enabled=test_config_data.get("enabled", True),
                        test_type=test_config_data.get("test_type", "python"),
                        source_file=str(hss_file),
                    )

                    strategy_class = get_strategy_class(test_config.strategy)
                    test_case = test_config.to_test_case(
                        test_manager.ros_dir, strategy_class
                    )
                    all_test_cases.append(test_case)

        except Exception as e:
            typer.echo(f"⚠️  Warning: Error loading {hss_file.name}: {e}")

    if not all_test_cases:
        typer.echo("❌ No enabled tests found in the specified directory")
        return

    typer.echo(f"🚀 Running {len(all_test_cases)} tests from directory")

    # Check if any tests need ROS
    needs_ros = any(
        isinstance(test_case.strategy, (RosTestStrategy, ScriptTestStrategy))
        or any(
            ros_keyword in str(test_case.path)
            for ros_keyword in [
                "autonomy",
                "mission_planner",
                "api_server",
                "controllers",
                "cv_publishers",
            ]
        )
        for test_case in all_test_cases
    )

    if needs_ros:
        test_manager._start_roscore(env)

    # Execute all test cases
    test_manager.execute_test_cases(all_test_cases, env)


def _run_specific_test_by_name(
    test_name: str,
    config_loader: DistributedTestConfigLoader,
    test_manager: HydrusTestManager,
    env: Dict[str, str],
):
    """Run a specific test by searching for it by name across all .hss files."""
    typer.echo(f"🔍 Searching for test: {test_name}")

    # Search across all test types
    unit_tests = config_loader.load_unit_tests()
    integration_tests = config_loader.load_integration_tests()
    script_tests = config_loader.load_script_tests()

    all_tests = unit_tests.tests + integration_tests.tests + script_tests.tests

    # Find the matching test
    matching_test = None
    for test_config in all_tests:
        if test_config.name.lower() == test_name.lower():
            matching_test = test_config
            break

    if not matching_test:
        typer.echo(f"❌ Test '{test_name}' not found.")

        # Show available tests
        typer.echo("\nAvailable tests:")
        for test_config in all_tests:
            if test_config.enabled:
                strategy_icon = (
                    "🔬"
                    if "unit" in test_config.strategy.lower()
                    else "🔗"
                    if "ros" in test_config.strategy.lower()
                    else "📜"
                )
                typer.echo(f"  {strategy_icon} {test_config.name}")

        raise typer.Exit(1)

    if not matching_test.enabled:
        typer.echo(f"❌ Test '{test_name}' is disabled.")
        raise typer.Exit(1)

    typer.echo(f"✅ Found test: {matching_test.name}")
    typer.echo(f"📝 Description: {matching_test.description}")

    # Convert to TestCase and execute
    strategy_class = get_strategy_class(matching_test.strategy)
    test_case = matching_test.to_test_case(test_manager.ros_dir, strategy_class)

    # Check if test needs ROS
    needs_ros = isinstance(
        test_case.strategy, (RosTestStrategy, ScriptTestStrategy)
    ) or any(
        ros_keyword in str(test_case.path)
        for ros_keyword in [
            "autonomy",
            "mission_planner",
            "api_server",
            "controllers",
            "cv_publishers",
        ]
    )

    if needs_ros:
        test_manager._start_roscore(env)

    # Execute the test
    test_manager.execute_test_cases([test_case], env)


# === Main Command ===


@test_app.command()
def run(
    target: str = typer.Argument(".", help="Directory path or test name to run"),
    debug: bool = typer.Option(False, "--debug", "-d", help="Enable debug mode"),
    volume: bool = typer.Option(
        False, "--volume", "-v", help="Use volume directory for tests"
    ),
):
    """Run tests automatically detecting the type and scope based on the target."""
    global _volume_mode
    _volume_mode = volume

    config_loader = DistributedTestConfigLoader()
    test_manager = HydrusTestManager(volume=_volume_mode, debug_mode=debug)
    env = os.environ.copy()

    # Get the repository root
    hydrus_root = os.environ.get("HYDRUS_ROOT")
    if hydrus_root:
        repo_root = Path(hydrus_root)
    else:
        building_path = get_building_path(_volume_mode)
        repo_root = building_path.parent

    try:
        test_manager._build_workspace()

        # Determine if target is a directory or test name
        if _is_directory_target(target, repo_root):
            # Handle directory target
            _run_tests_in_directory(target, repo_root, config_loader, test_manager, env)
        else:
            # Handle test name target
            _run_specific_test_by_name(target, config_loader, test_manager, env)

    finally:
        test_manager._cleanup_roscore()

    exit_code = test_manager._print_summary()
    raise typer.Exit(exit_code)
