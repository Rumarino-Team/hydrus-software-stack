#!/usr/bin/env python3
"""
test_config_loader.py

Distributed test configuration loader for the Hydrus test system.
Searches for .hss files throughout the repository and loads test configurations.
"""

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import yaml


@dataclass
class TestConfig:
    name: str
    path: str
    args: List[str]
    timeout: int
    infinite_loop: bool
    description: str
    strategy: str
    enabled: bool
    test_type: str
    source_file: Optional[str] = None  # Track which .hss file this came from
    dependencies: Optional[List[str]] = None  # Explicit dependencies

    def to_test_case(self, ros_dir: Path, strategy_class):
        """Convert TestConfig to TestCase object."""
        from .test_cases import TestCase

        # Handle absolute vs relative paths
        if self.path.startswith("/"):
            test_path = Path(self.path)
        else:
            # If source_file is available, make path relative to its directory
            if self.source_file:
                source_dir = Path(self.source_file).parent
                test_path = source_dir / self.path
            else:
                # Fallback to old behavior
                test_path = ros_dir / "src/hydrus-software-stack" / self.path

        return TestCase(
            name=self.name,
            path=test_path,
            args=self.args,
            timeout=self.timeout,
            infinite_loop=self.infinite_loop,
            description=self.description,
            strategy=strategy_class(),
        )


@dataclass
class TestSuite:
    name: str
    description: str
    strategy: str
    tests: List[TestConfig]
    source_files: List[str]  # Track which files contributed to this suite


class DistributedTestConfigLoader:
    """
    Loads test configurations from .hss files distributed throughout the repository.
    Searches for .hss files in specific directories and aggregates them by test type.
    """

    def __init__(self, repository_root: Optional[Path] = None):
        # First try to get root from environment variable
        hydrus_root = os.environ.get("HYDRUS_ROOT")
        if hydrus_root:
            self.repository_root = Path(hydrus_root)
            print(f"Using HYDRUS_ROOT environment variable: {self.repository_root}")
        elif repository_root:
            self.repository_root = repository_root
            print(f"Using provided repository root: {self.repository_root}")
        else:
            # Fallback: try to detect based on the location of this script
            script_path = Path(__file__).resolve()
            # Go up from scripts/commands/test_config_loader.py to find the project root
            potential_root = script_path.parent.parent.parent
            if (potential_root / "hydrus-cli").exists() or (
                potential_root / "autonomy"
            ).exists():
                self.repository_root = potential_root
                print(f"Auto-detected repository root: {self.repository_root}")
            else:
                raise RuntimeError(
                    "Could not determine repository root. Please set HYDRUS_ROOT environment variable."
                )

        self.search_paths = [
            "autonomy",
            "devices",
            "simulator",
            "scripts",
            "test_configs",  # Keep existing central configs
            "test",  # Root test directory
        ]

    def find_hss_files(self) -> List[Path]:
        """Find all .hss files in the repository."""
        hss_files = []

        for search_path in self.search_paths:
            search_dir = self.repository_root / search_path
            if search_dir.exists():
                # Recursively find all .hss files
                hss_files.extend(search_dir.rglob("*.hss"))

        return hss_files

    def _load_hss_file(self, file_path: Path) -> Dict:
        """Load an .hss file and return its contents."""
        try:
            with open(file_path, "r") as file:
                data = yaml.safe_load(file)
                return data if data is not None else {}
        except Exception as e:
            print(f"Warning: Failed to load .hss file {file_path}: {e}")
            return {}

    def _parse_test_configs(self, data: Dict, source_file: Path) -> List[TestConfig]:
        """Parse test configurations from .hss data."""
        test_configs = []

        for test_data in data.get("tests", []):
            test_config = TestConfig(
                name=test_data["name"],
                path=test_data["path"],
                args=test_data.get("args", []),
                timeout=test_data.get("timeout", 60),
                infinite_loop=test_data.get("infinite_loop", False),
                description=test_data.get("description", ""),
                strategy=test_data.get("strategy", "UnitTestStrategy"),
                enabled=test_data.get("enabled", True),
                test_type=test_data.get("test_type", "python"),
                source_file=str(source_file),
                dependencies=test_data.get("dependencies", None),
            )
            test_configs.append(test_config)

        return test_configs

    def load_tests_by_strategy(self, target_strategy: str) -> TestSuite:
        """Load all tests that match a specific strategy from distributed .hss files."""
        hss_files = self.find_hss_files()
        all_test_configs = []
        source_files = []

        suite_name = f"{target_strategy.title()} Tests"
        suite_description = (
            f"Tests loaded from distributed .hss files with strategy: {target_strategy}"
        )

        for hss_file in hss_files:
            try:
                data = self._load_hss_file(hss_file)
                if not data:
                    continue

                # Check if this file contains tests for our target strategy
                file_strategy = data.get("strategy", "")
                test_configs = self._parse_test_configs(data, hss_file)

                # Filter tests that match our target strategy (either file-level or test-level)
                matching_tests = []
                for test_config in test_configs:
                    if (
                        file_strategy == target_strategy
                        or test_config.strategy.lower().replace("teststrategy", "")
                        == target_strategy.lower().replace("teststrategy", "")
                        or test_config.test_type == target_strategy
                    ):
                        matching_tests.append(test_config)

                if matching_tests:
                    all_test_configs.extend(matching_tests)
                    source_files.append(str(hss_file))

                    # Use the first file's metadata if available
                    if data.get("name") and len(all_test_configs) == len(
                        matching_tests
                    ):
                        suite_name = data.get("name", suite_name)
                        suite_description = data.get("description", suite_description)

            except Exception as e:
                print(f"Warning: Error processing {hss_file}: {e}")
                continue

        return TestSuite(
            name=str(suite_name),
            description=str(suite_description),
            strategy=target_strategy,
            tests=all_test_configs,
            source_files=source_files,
        )

    def load_unit_tests(self) -> TestSuite:
        """Load unit test configurations from distributed .hss files."""
        return self.load_tests_by_strategy("unit")

    def load_integration_tests(self) -> TestSuite:
        """Load integration test configurations from distributed .hss files."""
        return self.load_tests_by_strategy("integration")

    def load_script_tests(self) -> TestSuite:
        """Load script test configurations from distributed .hss files."""
        return self.load_tests_by_strategy("script")

    def get_enabled_tests(self, test_suite: TestSuite) -> List[TestConfig]:
        """Filter and return only enabled tests."""
        return [test for test in test_suite.tests if test.enabled]

    def validate_test_config(self, test_config: TestConfig) -> bool:
        """Validate that a test configuration is valid."""
        # Handle absolute vs relative paths
        if test_config.path.startswith("/"):
            test_path = Path(test_config.path)
        else:
            if test_config.source_file:
                source_dir = Path(test_config.source_file).parent
                test_path = source_dir / test_config.path
            else:
                test_path = (
                    self.repository_root
                    / "src/hydrus-software-stack"
                    / test_config.path
                )

        if not test_path.exists():
            print(f"Warning: Test file does not exist: {test_path}")
            return False

        if test_config.timeout <= 0:
            print(
                f"Warning: Invalid timeout for test {test_config.name}: {test_config.timeout}"
            )
            return False

        return True

    def list_all_hss_files(self) -> Dict[str, List[Path]]:
        """List all .hss files organized by directory."""
        hss_files = self.find_hss_files()
        organized = {}

        for hss_file in hss_files:
            # Get relative path from repository root
            rel_path = hss_file.relative_to(self.repository_root)
            directory = str(rel_path.parent)

            if directory not in organized:
                organized[directory] = []
            organized[directory].append(rel_path)

        return organized


# Backward compatibility class
class TestConfigLoader(DistributedTestConfigLoader):
    """Legacy class name for backward compatibility."""

    def __init__(self, config_dir: Path):
        # Convert old usage to new usage
        # Assume config_dir is in repository, find repository root
        repository_root = config_dir
        while repository_root.parent != repository_root:
            if (repository_root / ".git").exists() or (
                repository_root / "setup.py"
            ).exists():
                break
            repository_root = repository_root.parent

        super().__init__(repository_root)


def get_strategy_class(strategy_name: str):
    """Get the strategy class based on the strategy name."""
    from .test_cases import RosTestStrategy, ScriptTestStrategy, UnitTestStrategy

    strategy_map = {
        "UnitTestStrategy": UnitTestStrategy,
        "RosTestStrategy": RosTestStrategy,
        "ScriptTestStrategy": ScriptTestStrategy,
    }

    return strategy_map.get(strategy_name, UnitTestStrategy)
