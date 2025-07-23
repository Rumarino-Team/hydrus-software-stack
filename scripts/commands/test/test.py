import os
from pathlib import Path
from typing import Dict

import typer
import yaml

from ..utils import get_building_path
from .test_cases import RosTestStrategy, ScriptTestStrategy
from .test_config_loader import (
    DistributedTestConfigLoader,
    TestConfig,
    get_strategy_class,
)
from .test_manager import HydrusTestManager

# Create the test app
test_app = typer.Typer()

# Global state to store volume setting
_volume_mode = False


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
