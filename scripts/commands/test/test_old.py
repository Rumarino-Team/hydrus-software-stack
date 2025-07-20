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
    TestConfigLoader,
    get_strategy_class,
)
from .test_logger import TestLogManager, run_command_with_logging

test_app = typer.Typer()

# Global state to store volume setting
_volume_mode = False


@test_app.callback()
def test_callback(
    volume: bool = typer.Option(
        False, "--volume", "-v", help="Use volume directory for tests"
    )
):
    """Test suite management and execution"""
    global _volume_mode
    _volume_mode = volume


# === Strategy Pattern for Test Exec        typer.echo(f"🔍 Searching for .hss                     typer.echo(            typer.echo(f"✨ Total tests found: {total_tests}")"  ✨ {len(tests_to_show)} tests found")iles in: {search_dir}")tion ===


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


# === Test Builders ===


class TestBuilder(ABC):
    @abstractmethod
    def build_tests(self) -> List[TestCase]:
        pass


class UnitTestBuilder(TestBuilder):
    def __init__(self, ros_dir: Path):
        self.ros_dir = ros_dir
        self.config_loader = DistributedTestConfigLoader()

    def build_tests(self) -> List[TestCase]:
        # Load unit tests from distributed .hss files
        unit_test_suite = self.config_loader.load_unit_tests()
        enabled_tests = self.config_loader.get_enabled_tests(unit_test_suite)

        test_cases = []
        for test_config in enabled_tests:
            strategy_class = get_strategy_class(test_config.strategy)
            test_case = test_config.to_test_case(self.ros_dir, strategy_class)
            test_cases.append(test_case)

        return test_cases


class RosIntegrationTestBuilder(TestBuilder):
    def __init__(self, ros_dir: Path):
        self.ros_dir = ros_dir
        self.config_loader = DistributedTestConfigLoader()

    def build_tests(self) -> List[TestCase]:
        # Load integration tests from distributed .hss files
        integration_test_suite = self.config_loader.load_integration_tests()
        enabled_tests = self.config_loader.get_enabled_tests(integration_test_suite)

        test_cases = []
        for test_config in enabled_tests:
            strategy_class = get_strategy_class(test_config.strategy)
            test_case = test_config.to_test_case(self.ros_dir, strategy_class)
            test_cases.append(test_case)

        return test_cases


class ScriptTestBuilder(TestBuilder):
    def __init__(self, ros_dir: Path):
        self.ros_dir = ros_dir
        self.config_loader = DistributedTestConfigLoader()

    def build_tests(self) -> List[TestCase]:
        # Load script tests from distributed .hss files
        script_test_suite = self.config_loader.load_script_tests()
        enabled_tests = self.config_loader.get_enabled_tests(script_test_suite)

        test_cases = []
        for test_config in enabled_tests:
            strategy_class = get_strategy_class(test_config.strategy)
            test_case = test_config.to_test_case(self.ros_dir, strategy_class)
            test_cases.append(test_case)

        return test_cases


# === Test Director ===


class TestDirector:
    def __init__(self, builder: TestBuilder):
        self.builder = builder

    def construct_tests(self) -> List[TestCase]:
        return self.builder.build_tests()


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
        self.current_test_log = None

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
            print(f"   Error: {e}")
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
        cmake_command = "source /opt/ros/noetic/setup.bash && catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.9 -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.9.so"

        cmake_args = ["bash", "-c", cmake_command]

        try:
            self._run_command(cmake_args, check=False, cwd=self.ros_dir, env=env)
        except subprocess.CalledProcessError:
            print("Build failed, but continuing...")

    def _is_process_running(self, pid: int) -> bool:
        """Check if a process is running"""
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False

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

    def _signal_handler(self, signum, frame):
        """Handle interrupt signals gracefully"""
        print(f"\n🛑 Received signal {signum}. Cleaning up...")
        self._cleanup_roscore()
        print("Cleanup completed. Exiting.")
        sys.exit(1)

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

    def _cleanup_roscore(self):
        """Clean up roscore process"""
        if self.roscore_pid and self._is_process_running(self.roscore_pid):
            print("Cleaning up roscore...")
            try:
                os.kill(self.roscore_pid, signal.SIGTERM)
                time.sleep(2)

                # Force kill if still running
                if self._is_process_running(self.roscore_pid):
                    os.kill(self.roscore_pid, signal.SIGKILL)
                    print("Forcefully killed roscore")
                else:
                    print("Roscore stopped gracefully")

            except OSError:
                pass  # Process was already dead

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

    def _run_unit_tests(self, env: Dict[str, str]):
        """Run unit tests using strategy pattern"""
        print("\n🔬 Running Unit Tests...")

        # Create unit test builder and director
        unit_builder = UnitTestBuilder(self.ros_dir)
        director = TestDirector(unit_builder)
        unit_tests = director.construct_tests()

        # Execute tests
        self.execute_test_cases(unit_tests, env)

    def _run_ros_integration_tests(self, env: Dict[str, str]):
        """Run ROS integration tests using strategy pattern"""
        print("\n🔗 Running ROS Integration Tests...")

        # Start roscore for ROS tests
        self._start_roscore(env)

        # Create ROS test builder and director
        ros_builder = RosIntegrationTestBuilder(self.ros_dir)
        director = TestDirector(ros_builder)
        ros_tests = director.construct_tests()

        # Execute tests
        self.execute_test_cases(ros_tests, env)

    def _run_script_tests(self, env: Dict[str, str]):
        """Run script tests using strategy pattern"""
        print("\n📜 Running Script Tests...")

        # Create script test builder and director
        script_builder = ScriptTestBuilder(self.ros_dir)
        director = TestDirector(script_builder)
        script_tests = director.construct_tests()

        # Execute tests
        self.execute_test_cases(script_tests, env)

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

    def _find_integration_test_by_name(self, test_name: str) -> Optional[TestCase]:
        """Find an integration test by its name"""
        ros_builder = RosIntegrationTestBuilder(self.ros_dir)
        director = TestDirector(ros_builder)
        ros_tests = director.construct_tests()

        for test in ros_tests:
            if test.name.lower() == test_name.lower():
                return test
        return None

    def _list_integration_test_names(self) -> List[str]:
        """Get a list of all integration test names"""
        ros_builder = RosIntegrationTestBuilder(self.ros_dir)
        director = TestDirector(ros_builder)
        ros_tests = director.construct_tests()

        return [test.name for test in ros_tests]

    def _run_specific_integration_test(self, test_name: str, env: Dict[str, str]):
        """Run a specific integration test by name"""
        test_case = self._find_integration_test_by_name(test_name)

        if not test_case:
            available_tests = self._list_integration_test_names()
            print(f"❌ Integration test '{test_name}' not found.")
            print("Available integration tests:")
            for name in available_tests:
                print(f"  • {name}")
            return

        print(f"\n🔗 Running specific integration test: {test_name}")

        # Start roscore for ROS tests
        self._start_roscore(env)

        # Execute the specific test
        self.execute_test_cases([test_case], env)


# === Typer Commands ===


@test_app.command("all")
def run_all_tests(
    debug: bool = typer.Option(False, "--debug", "-d", help="Enable debug mode"),
):
    """Run all tests in the Hydrus software stack."""
    typer.echo("🚀 Running all tests in the Hydrus software stack...")

    test_manager = HydrusTestManager(volume=_volume_mode, debug_mode=debug)
    test_manager.main()


@test_app.command("unit")
def run_unit_tests_cmd(
    debug: bool = typer.Option(False, "--debug", "-d", help="Enable debug mode"),
):
    """Run only unit tests."""
    typer.echo("🔬 Running unit tests...")

    test_manager = HydrusTestManager(volume=_volume_mode, debug_mode=debug)
    env = os.environ.copy()

    try:
        test_manager._build_workspace()
        test_manager._run_unit_tests(env)
    finally:
        test_manager._cleanup_roscore()

    exit_code = test_manager._print_summary()
    raise typer.Exit(exit_code)


@test_app.command("integration")
def run_integration_tests_cmd(
    debug: bool = typer.Option(False, "--debug", "-d", help="Enable debug mode"),
):
    """Run only ROS integration tests."""
    typer.echo("🔗 Running ROS integration tests...")

    test_manager = HydrusTestManager(volume=_volume_mode, debug_mode=debug)
    env = os.environ.copy()

    try:
        test_manager._build_workspace()
        test_manager._run_ros_integration_tests(env)
    finally:
        test_manager._cleanup_roscore()

    exit_code = test_manager._print_summary()
    raise typer.Exit(exit_code)


@test_app.command("integration-test")
def run_specific_integration_test(
    test_name: str = typer.Argument(..., help="Name of the integration test to run"),
    debug: bool = typer.Option(False, "--debug", "-d", help="Enable debug mode"),
):
    """Run a specific integration test by name."""
    typer.echo(f"🔗 Running specific integration test: {test_name}")

    test_manager = HydrusTestManager(volume=_volume_mode, debug_mode=debug)
    env = os.environ.copy()

    try:
        test_manager._build_workspace()
        test_manager._run_specific_integration_test(test_name, env)
    finally:
        test_manager._cleanup_roscore()

    exit_code = test_manager._print_summary()
    raise typer.Exit(exit_code)


@test_app.command("scripts")
def run_script_tests_cmd(
    debug: bool = typer.Option(False, "--debug", "-d", help="Enable debug mode"),
):
    """Run only script tests."""
    typer.echo("📜 Running script tests...")

    test_manager = HydrusTestManager(volume=_volume_mode, debug_mode=debug)
    env = os.environ.copy()

    try:
        test_manager._build_workspace()
        test_manager._run_script_tests(env)
    finally:
        test_manager._cleanup_roscore()

    exit_code = test_manager._print_summary()
    raise typer.Exit(exit_code)


@test_app.command("custom")
def run_custom_test(
    test_name: str = typer.Argument(..., help="Name of the test"),
    test_path: str = typer.Argument(..., help="Path to the test file"),
    debug: bool = typer.Option(False, "--debug", "-d", help="Enable debug mode"),
    timeout: int = typer.Option(300, "--timeout", "-t", help="Test timeout in seconds"),
    args: List[str] = typer.Option(
        [], "--args", "-a", help="Additional arguments for the test"
    ),
):
    """Run a custom test with specified parameters."""
    typer.echo(f"🧪 Running custom test: {test_name}")

    test_manager = HydrusTestManager(volume=_volume_mode, debug_mode=debug)
    env = os.environ.copy()

    # Create a custom test case
    custom_test = TestCase(
        name=test_name,
        path=Path(test_path),
        args=args,
        timeout=timeout,
        infinite_loop=False,
        description=f"Custom test: {test_name}",
        strategy=UnitTestStrategy(),
    )

    try:
        test_manager._build_workspace()
        test_manager.execute_test_cases([custom_test], env)
    finally:
        test_manager._cleanup_roscore()

    exit_code = test_manager._print_summary()
    raise typer.Exit(exit_code)


@test_app.command("query")
def query_tests_in_directory(
    directory: str = typer.Argument(
        ".", help="Directory to search for .hss files (relative to repo root)"
    ),
    test_type: str = typer.Option(
        "all",
        "--type",
        "-t",
        help="Filter by test type: unit, integration, script, or all",
    ),
    show_details: bool = typer.Option(
        False, "--details", "-d", help="Show detailed test information"
    ),
):
    """Query and display tests defined in .hss files within a directory."""
    try:
        config_loader = DistributedTestConfigLoader()

        # Get the repository root from HYDRUS_ROOT environment variable
        hydrus_root = os.environ.get("HYDRUS_ROOT")
        if hydrus_root:
            repo_root = Path(hydrus_root)
        else:
            # Fallback to building path method
            building_path = get_building_path(_volume_mode)
            repo_root = building_path.parent

        typer.echo(f"📁 Repository root: {repo_root}")

        # Handle different directory specifications
        if directory == "." or directory == "./":
            # Use the current working directory when "." is specified
            search_dir = Path.cwd()
            # But ensure it's within the repo
            try:
                search_dir.relative_to(repo_root)
            except ValueError:
                # If current directory is not within repo, use repo root
                search_dir = repo_root
        elif directory.startswith("/"):
            # Absolute path - make sure it's within repo bounds
            abs_path = Path(directory)
            if not str(abs_path).startswith(str(repo_root)):
                typer.echo(f"❌ Directory must be within repository: {repo_root}")
                raise typer.Exit(1)
            search_dir = abs_path
        else:
            # Relative path from repo root
            search_dir = repo_root / directory

        if not search_dir.exists():
            typer.echo(f"❌ Directory not found: {search_dir}")
            raise typer.Exit(1)

        typer.echo(f"� Searching for .hss files in: {search_dir}")

        # Find all .hss files in the directory and subdirectories
        hss_files = list(search_dir.rglob("*.hss"))

        if not hss_files:
            typer.echo(f"❌ No .hss files found in {search_dir}")
            return

        typer.echo(f"📁 Found {len(hss_files)} .hss files")
        typer.echo()

        total_tests = 0

        for hss_file in sorted(hss_files):
            rel_path = hss_file.relative_to(repo_root)
            typer.echo(f"📄 {rel_path}")

            try:
                # Load and parse YAML directly to avoid circular import issues
                with open(hss_file, "r") as f:
                    hss_content = yaml.safe_load(f)

                if not hss_content or "tests" not in hss_content:
                    typer.echo("  📊 No tests found in this file")
                    continue

                tests = hss_content["tests"]

                # Filter tests by type if specified
                tests_to_show = []
                for test in tests:
                    strategy = test.get("strategy", "").lower()
                    if test_type == "all":
                        tests_to_show.append(test)
                    elif test_type == "unit" and ("unit" in strategy):
                        tests_to_show.append(test)
                    elif test_type == "integration" and (
                        "ros" in strategy or "integration" in strategy
                    ):
                        tests_to_show.append(test)
                    elif test_type == "script" and ("script" in strategy):
                        tests_to_show.append(test)

                if tests_to_show:
                    typer.echo(f"  � {len(tests_to_show)} tests found")
                    total_tests += len(tests_to_show)

                    for test in tests_to_show:
                        enabled = test.get("enabled", True)
                        status = "✅ Enabled" if enabled else "❌ Disabled"
                        strategy = test.get("strategy", "Unknown")

                        strategy_icon = (
                            "🔬"
                            if "unit" in strategy.lower()
                            else "🔗"
                            if "ros" in strategy.lower()
                            else "📜"
                        )

                        test_name = test.get("name", "Unnamed Test")

                        if show_details:
                            typer.echo(f"    {strategy_icon} {test_name} ({status})")
                            typer.echo(
                                f"      📝 {test.get('description', 'No description')}"
                            )
                            typer.echo(f"      📁 Path: {test.get('path', 'No path')}")
                            typer.echo(f"      ⏱️  Timeout: {test.get('timeout', 60)}s")
                            if test.get("args"):
                                typer.echo(f"      🔧 Args: {test['args']}")
                            typer.echo()
                        else:
                            typer.echo(f"    {strategy_icon} {test_name} ({status})")
                else:
                    typer.echo("  📊 No tests match the specified type filter")

            except Exception as e:
                typer.echo(f"  ❌ Error loading {hss_file.name}: {e}")

            typer.echo()

        if total_tests > 0:
            typer.echo(f"� Total tests found: {total_tests}")
        else:
            typer.echo("📈 No tests found matching the criteria")

    except Exception as e:
        typer.echo(f"❌ Error querying tests: {e}")
        raise typer.Exit(1)


@test_app.command("validate")
def validate_test_environment():
    """Validate the test environment setup."""
    typer.echo("🔍 Validating test environment...")

    test_manager = HydrusTestManager(volume=_volume_mode)

    # Check ROS workspace
    if not test_manager.ros_dir.exists():
        typer.echo(f"❌ ROS workspace not found: {test_manager.ros_dir}")
        raise typer.Exit(1)
    else:
        typer.echo(f"✅ ROS workspace found: {test_manager.ros_dir}")

    # Check test logs directory
    if not test_manager.test_logs_dir.exists():
        typer.echo(f"❌ Test logs directory not found: {test_manager.test_logs_dir}")
        raise typer.Exit(1)
    else:
        typer.echo(f"✅ Test logs directory found: {test_manager.test_logs_dir}")

    # Check for test files
    unit_builder = UnitTestBuilder(test_manager.ros_dir)
    director = TestDirector(unit_builder)
    unit_tests = director.construct_tests()

    missing_tests = []
    for test in unit_tests:
        if not test.path.exists():
            missing_tests.append(test.path)

    if missing_tests:
        typer.echo("⚠️  Missing test files:")
        for path in missing_tests:
            typer.echo(f"  • {path}")
    else:
        typer.echo("✅ All test files found")

    typer.echo("✅ Test environment validation complete")
