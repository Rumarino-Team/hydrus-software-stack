from abc import ABC, abstractmethod
from pathlib import Path
from typing import Dict, List

# === Strategy Pattern for Test Execution ===


class TestExecutionStrategy(ABC):
    @abstractmethod
    def execute(self, test_case: "TestCase", env: Dict, runner):
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
