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

from .utils import get_building_path

test_app = typer.Typer()


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


# === Test Builders ===


class TestBuilder(ABC):
    @abstractmethod
    def build_tests(self) -> List[TestCase]:
        pass


class UnitTestBuilder(TestBuilder):
    def __init__(self, ros_dir: Path):
        self.ros_dir = ros_dir

    def build_tests(self) -> List[TestCase]:
        return [
            TestCase(
                name="Tagging Mission Unit Test",
                path=self.ros_dir
                / "src/hydrus-software-stack/autonomy/src/mission_planner/tagging_mission_test.py",
                args=[],
                timeout=30,
                infinite_loop=False,
                description="Unit test for tagging mission",
                strategy=UnitTestStrategy(),
            )
        ]


class RosIntegrationTestBuilder(TestBuilder):
    def __init__(self, ros_dir: Path):
        self.ros_dir = ros_dir

    def build_tests(self) -> List[TestCase]:
        return [
            TestCase(
                name="Controller Tests",
                path=Path("controller.test"),
                args=[],
                timeout=600,
                infinite_loop=False,
                description="ROS controller integration tests using rostest",
                strategy=RosTestStrategy(),
            ),
            TestCase(
                name="Slalom Integration Tests",
                path=self.ros_dir
                / "src/hydrus-software-stack/autonomy/src/mission_planner/test_slalom_integration.py",
                args=[],
                timeout=300,
                infinite_loop=False,
                description="Slalom mission integration tests",
                strategy=UnitTestStrategy(),
            ),
            TestCase(
                name="Gate Mission Tests",
                path=self.ros_dir
                / "src/hydrus-software-stack/autonomy/src/mission_planner/test_gate_mission.py",
                args=[],
                timeout=300,
                infinite_loop=False,
                description="Gate mission functionality tests",
                strategy=UnitTestStrategy(),
            ),
            TestCase(
                name="DVL Driver Tests",
                path=self.ros_dir
                / "src/hydrus-software-stack/DVL/Wayfinder/driver_test.py",
                args=[],
                timeout=60,
                infinite_loop=False,
                description="DVL driver functionality tests",
                strategy=UnitTestStrategy(),
            ),
        ]


class ScriptTestBuilder(TestBuilder):
    def __init__(self, ros_dir: Path):
        self.ros_dir = ros_dir

    def build_tests(self) -> List[TestCase]:
        return [
            TestCase(
                name="API Server",
                path=self.ros_dir
                / "src/hydrus-software-stack/autonomy/src/api_server.py",
                args=[],
                timeout=2,  # Increased slightly for safety
                infinite_loop=True,
                description="API Server script that runs indefinitely to handle API requests.",
                strategy=ScriptTestStrategy(),
            ),
            TestCase(
                name="Controllers Node",
                path=self.ros_dir
                / "src/hydrus-software-stack/autonomy/src/controllers.py",
                args=[],
                timeout=2,
                infinite_loop=True,
                description="Controllers Node script that runs indefinitely. Handles controller logic for the robot.",
                strategy=ScriptTestStrategy(),
            ),
            TestCase(
                name="Computer Vision Node",
                path=self.ros_dir
                / "src/hydrus-software-stack/autonomy/src/cv_publishers.py",
                args=[],
                timeout=2,
                infinite_loop=True,
                description="Computer vision Node script that runs indefinitely for image processing.",
                strategy=ScriptTestStrategy(),
            ),
            TestCase(
                name="Controller Monitor Node",
                path=self.ros_dir
                / "src/hydrus-software-stack/autonomy/scripts/controller/controller_monitor.py",
                args=[],
                timeout=2,
                infinite_loop=True,
                description="Controller Monitor Node script that runs indefinitely to manage missions.",
                strategy=ScriptTestStrategy(),
            ),
        ]


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

        # Logging setup
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
        """Run a unified test using the strategy pattern"""
        print("")
        print("----------------------------------------")
        print(f"Running test: {test_name}")
        print(f"Description: {description}")
        print("----------------------------------------")

        self.total_tests += 1

        # Create individual test log file
        test_log_file = (
            self.test_logs_dir / f"{test_name.replace(' ', '_').lower()}.log"
        )

        # Log test start
        self._log_message(f"=== Starting Test: {test_name} ===")
        self._log_message(f"Description: {description}")
        self._log_message(f"Test path: {test_path}")
        self._log_message(f"Args: {args}")
        self._log_message(f"Timeout: {timeout}s")
        self._log_message(f"Infinite loop: {infinite_loop}")

        try:
            if test_path.suffix == ".py":
                # Python test
                cmd = ["python3", str(test_path)] + args
            else:
                # Shell script or other executable
                cmd = [str(test_path)] + args

            # Log the command
            cmd_str = " ".join(cmd)
            self._log_message(f"Command: {cmd_str}")

            # Execute test with timeout unless it's an infinite loop test
            test_timeout = timeout if not infinite_loop else timeout

            with open(test_log_file, "w") as f:
                try:
                    subprocess.run(
                        cmd,
                        timeout=test_timeout,
                        check=True,
                        stdout=f,
                        stderr=subprocess.STDOUT,
                        env=env,
                        cwd=self.ros_dir,
                        text=True,
                    )

                    if infinite_loop:
                        # For infinite loop tests, timeout is expected
                        print(f"✅ PASSED: {test_name} (ran for expected {timeout}s)")
                        self.passed_tests += 1
                        self._log_message(f"Test passed (infinite loop): {test_name}")
                    else:
                        print(f"✅ PASSED: {test_name}")
                        self.passed_tests += 1
                        self._log_message(f"Test passed: {test_name}")

                except subprocess.TimeoutExpired:
                    if infinite_loop:
                        # This is expected for infinite loop tests
                        print(f"✅ PASSED: {test_name} (timed out as expected)")
                        self.passed_tests += 1
                        self._log_message(
                            f"Test passed (expected timeout): {test_name}"
                        )
                    else:
                        print(f"⏰ TIMEOUT: {test_name} (exceeded {timeout}s)")
                        self.failed_tests += 1
                        self._log_message(f"Test timed out: {test_name}")

        except subprocess.CalledProcessError as e:
            print(f"❌ FAILED: {test_name} (exit code: {e.returncode})")
            self.failed_tests += 1
            self._log_message(f"Test failed: {test_name} with exit code {e.returncode}")

        except FileNotFoundError:
            print(f"❌ FAILED: {test_name} (test file not found: {test_path})")
            cmd_str = " ".join([str(test_path)] + args)

            # Write failure details to individual test log
            with open(test_log_file, "w") as f:
                f.write("Test failed: File not found\n")
                f.write(f"Path: {test_path}\n")
                f.write(f"Command: {cmd_str}\n")

            self.failed_tests += 1

        # Log test completion
        self._log_message(f"=== Completed Test: {test_name} ===\n")

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
        """Print test results summary and return exit code"""
        print("")
        print("==========================================")
        print("TEST RESULTS SUMMARY")
        print("==========================================")
        print(f"Total tests run: {self.total_tests}")
        print(f"✅ Passed: {self.passed_tests}")
        print(f"❌ Failed: {self.failed_tests}")

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
            print(f"📁 Detailed logs available in: {self.test_logs_dir}")
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


# === Typer Commands ===


@test_app.command("all")
def run_all_tests(
    volume: bool = typer.Option(
        False, "--volume", "-v", help="Use volume directory for tests"
    ),
    debug: bool = typer.Option(False, "--debug", "-d", help="Enable debug mode"),
):
    """Run all tests in the Hydrus software stack."""
    typer.echo("🚀 Running all tests in the Hydrus software stack...")

    test_manager = HydrusTestManager(volume=volume, debug_mode=debug)
    test_manager.main()


@test_app.command("unit")
def run_unit_tests_cmd(
    volume: bool = typer.Option(
        False, "--volume", "-v", help="Use volume directory for tests"
    ),
    debug: bool = typer.Option(False, "--debug", "-d", help="Enable debug mode"),
):
    """Run only unit tests."""
    typer.echo("🔬 Running unit tests...")

    test_manager = HydrusTestManager(volume=volume, debug_mode=debug)
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
    volume: bool = typer.Option(
        False, "--volume", "-v", help="Use volume directory for tests"
    ),
    debug: bool = typer.Option(False, "--debug", "-d", help="Enable debug mode"),
):
    """Run only ROS integration tests."""
    typer.echo("🔗 Running ROS integration tests...")

    test_manager = HydrusTestManager(volume=volume, debug_mode=debug)
    env = os.environ.copy()

    try:
        test_manager._build_workspace()
        test_manager._run_ros_integration_tests(env)
    finally:
        test_manager._cleanup_roscore()

    exit_code = test_manager._print_summary()
    raise typer.Exit(exit_code)


@test_app.command("scripts")
def run_script_tests_cmd(
    volume: bool = typer.Option(
        False, "--volume", "-v", help="Use volume directory for tests"
    ),
    debug: bool = typer.Option(False, "--debug", "-d", help="Enable debug mode"),
):
    """Run only script tests."""
    typer.echo("📜 Running script tests...")

    test_manager = HydrusTestManager(volume=volume, debug_mode=debug)
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
    volume: bool = typer.Option(
        False, "--volume", "-v", help="Use volume directory for tests"
    ),
    debug: bool = typer.Option(False, "--debug", "-d", help="Enable debug mode"),
    timeout: int = typer.Option(300, "--timeout", "-t", help="Test timeout in seconds"),
    args: List[str] = typer.Option(
        [], "--args", "-a", help="Additional arguments for the test"
    ),
):
    """Run a custom test with specified parameters."""
    typer.echo(f"🧪 Running custom test: {test_name}")

    test_manager = HydrusTestManager(volume=volume, debug_mode=debug)
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


@test_app.command("list")
def list_available_tests(
    volume: bool = typer.Option(False, "--volume", "-v", help="Use volume directory")
):
    """List all available tests."""
    typer.echo("📋 Available tests:")

    test_manager = HydrusTestManager(volume=volume)

    # List unit tests
    unit_builder = UnitTestBuilder(test_manager.ros_dir)
    director = TestDirector(unit_builder)
    unit_tests = director.construct_tests()

    typer.echo("\n🔬 Unit Tests:")
    for test in unit_tests:
        typer.echo(f"  • {test.name}: {test.description}")

    # List ROS integration tests
    ros_builder = RosIntegrationTestBuilder(test_manager.ros_dir)
    director = TestDirector(ros_builder)
    ros_tests = director.construct_tests()

    typer.echo("\n🔗 ROS Integration Tests:")
    for test in ros_tests:
        typer.echo(f"  • {test.name}: {test.description}")

    # List script tests
    script_builder = ScriptTestBuilder(test_manager.ros_dir)
    director = TestDirector(script_builder)
    script_tests = director.construct_tests()

    typer.echo("\n📜 Script Tests:")
    for test in script_tests:
        typer.echo(f"  • {test.name}: {test.description}")


@test_app.command("validate")
def validate_test_environment(
    volume: bool = typer.Option(False, "--volume", "-v", help="Use volume directory")
):
    """Validate the test environment setup."""
    typer.echo("🔍 Validating test environment...")

    test_manager = HydrusTestManager(volume=volume)

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
        """Run a unified test using the strategy pattern"""
        print("")
        print("----------------------------------------")
        print(f"Running test: {test_name}")
        print(f"Description: {description}")
        print("----------------------------------------")

        self.total_tests += 1

        # Create individual test log file
        test_log_file = (
            self.test_logs_dir / f"{test_name.replace(' ', '_').lower()}.log"
        )

        # Log test start
        self._log_message(f"=== Starting Test: {test_name} ===")
        self._log_message(f"Description: {description}")
        self._log_message(f"Test path: {test_path}")
        self._log_message(f"Args: {args}")
        self._log_message(f"Timeout: {timeout}s")
        self._log_message(f"Infinite loop: {infinite_loop}")

        try:
            if test_path.suffix == ".py":
                # Python test
                cmd = ["python3", str(test_path)] + args
            else:
                # Shell script or other executable
                cmd = [str(test_path)] + args

            # Log the command
            cmd_str = " ".join(cmd)
            self._log_message(f"Command: {cmd_str}")

            # Execute test with timeout unless it's an infinite loop test
            test_timeout = None if infinite_loop else timeout

            with open(test_log_file, "w") as f:
                subprocess.run(
                    cmd,
                    timeout=test_timeout,
                    check=True,
                    stdout=f,
                    stderr=subprocess.STDOUT,
                    env=env,
                    cwd=self.ros_dir,
                    text=True,
                )

            print(f"✅ PASSED: {test_name}")
            self.passed_tests += 1
            self._log_message(f"Test passed: {test_name}")

        except subprocess.TimeoutExpired:
            print(f"⏰ TIMEOUT: {test_name} (exceeded {timeout}s)")
            self.failed_tests += 1
            self._log_message(f"Test timed out: {test_name}")

        except subprocess.CalledProcessError as e:
            print(f"❌ FAILED: {test_name} (exit code: {e.returncode})")
            self.failed_tests += 1
            self._log_message(f"Test failed: {test_name} with exit code {e.returncode}")

        except FileNotFoundError:
            print(f"❌ FAILED: {test_name} (test file not found: {test_path})")
            cmd_str = " ".join([str(test_path)] + args)

            # Write failure details to individual test log
            with open(test_log_file, "w") as f:
                f.write("Test failed: File not found\n")
                f.write(f"Path: {test_path}\n")
                f.write(f"Command: {cmd_str}\n")

            self.failed_tests += 1

        # Log test completion
        self._log_message(f"=== Completed Test: {test_name} ===\n")

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
        # Placeholder for script tests - can be extended later
        pass

    def _print_summary(self) -> int:
        """Print test results summary and return exit code"""
        print("")
        print("==========================================")
        print("TEST RESULTS SUMMARY")
        print("==========================================")
        print(f"Total tests run: {self.total_tests}")
        print(f"✅ Passed: {self.passed_tests}")
        print(f"❌ Failed: {self.failed_tests}")

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
            print(f"📁 Detailed logs available in: {self.test_logs_dir}")
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
