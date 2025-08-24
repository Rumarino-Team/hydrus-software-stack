#!/usr/bin/env python3
"""
test_logger.py

Advanced logging system for Hydrus test execution with console output and archive creation.
"""

import logging
import queue
import subprocess
import threading
import zipfile
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, TextIO


class TestLogCapture:
    """Captures and manages logs for individual test processes."""

    def __init__(self, test_name: str, log_dir: Path):
        self.test_name = test_name
        self.log_dir = log_dir
        self.log_file = log_dir / f"{self._sanitize_name(test_name)}.log"
        self.start_time: Optional[datetime] = None
        self.end_time: Optional[datetime] = None
        self.return_code: Optional[int] = None
        self.log_queue = queue.Queue()
        self.console_output = []

        # Ensure log directory exists
        self.log_dir.mkdir(parents=True, exist_ok=True)

        # Setup logging
        self.logger = logging.getLogger(f"test.{test_name}")
        self.logger.setLevel(logging.DEBUG)

        # File handler
        file_handler = logging.FileHandler(self.log_file, mode="w", encoding="utf-8")
        file_formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )
        file_handler.setFormatter(file_formatter)
        self.logger.addHandler(file_handler)

        # Console handler for immediate output
        console_handler = logging.StreamHandler()
        console_formatter = logging.Formatter(
            f"[{test_name}] %(levelname)s: %(message)s"
        )
        console_handler.setFormatter(console_formatter)
        self.logger.addHandler(console_handler)

    def _sanitize_name(self, name: str) -> str:
        """Sanitize test name for use as filename."""
        return "".join(c for c in name if c.isalnum() or c in (" ", "-", "_")).rstrip()

    def start_capture(self):
        """Start capturing logs for this test."""
        self.start_time = datetime.now()
        self.logger.info(f"Starting test: {self.test_name}")
        self.logger.info(f"Start time: {self.start_time.isoformat()}")

    def log_info(self, message: str):
        """Log an info message."""
        self.logger.info(message)
        self.console_output.append(f"INFO: {message}")

    def log_warning(self, message: str):
        """Log a warning message."""
        self.logger.warning(message)
        self.console_output.append(f"WARNING: {message}")

    def log_error(self, message: str):
        """Log an error message."""
        self.logger.error(message)
        self.console_output.append(f"ERROR: {message}")

    def log_debug(self, message: str):
        """Log a debug message."""
        self.logger.debug(message)
        self.console_output.append(f"DEBUG: {message}")

    def log_command_output(self, line: str):
        """Log output from a command."""
        # Remove ANSI color codes for log file
        import re

        clean_line = re.sub(r"\x1b\[[0-9;]*m", "", line.strip())
        if clean_line:
            self.logger.info(f"STDOUT: {clean_line}")
            self.console_output.append(clean_line)

    def log_command_error(self, line: str):
        """Log error output from a command."""
        import re

        clean_line = re.sub(r"\x1b\[[0-9;]*m", "", line.strip())
        if clean_line:
            self.logger.error(f"STDERR: {clean_line}")
            self.console_output.append(f"ERROR: {clean_line}")

    def end_capture(self, return_code: int = 0):
        """End capturing logs for this test."""
        self.end_time = datetime.now()
        self.return_code = return_code
        duration = (
            (self.end_time - self.start_time).total_seconds() if self.start_time else 0
        )

        self.logger.info(f"Test completed: {self.test_name}")
        self.logger.info(f"End time: {self.end_time.isoformat()}")
        self.logger.info(f"Duration: {duration:.2f} seconds")
        self.logger.info(f"Return code: {return_code}")

        if return_code == 0:
            self.logger.info("Test PASSED")
        else:
            self.logger.error("Test FAILED")

    def get_summary(self) -> Dict:
        """Get a summary of this test execution."""
        duration = 0
        if self.start_time and self.end_time:
            duration = (self.end_time - self.start_time).total_seconds()

        return {
            "test_name": self.test_name,
            "start_time": self.start_time.isoformat() if self.start_time else None,
            "end_time": self.end_time.isoformat() if self.end_time else None,
            "duration": duration,
            "return_code": self.return_code,
            "status": "PASSED" if self.return_code == 0 else "FAILED",
            "log_file": str(self.log_file),
            "log_size": self.log_file.stat().st_size if self.log_file.exists() else 0,
        }


class TestLogManager:
    """Manages logging for all test executions and creates archives."""

    def __init__(self, session_name: Optional[str] = None):
        self.session_name = (
            session_name or f"test_session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
        self.base_log_dir = Path.cwd() / "test_logs" / self.session_name
        self.test_captures: Dict[str, TestLogCapture] = {}
        self.session_start_time = datetime.now()
        self.session_end_time: Optional[datetime] = None

        # Create session log directory
        self.base_log_dir.mkdir(parents=True, exist_ok=True)

        # Setup session logger
        self.session_logger = logging.getLogger("test_session")
        self.session_logger.setLevel(logging.INFO)

        session_log_file = self.base_log_dir / "session.log"
        session_handler = logging.FileHandler(
            session_log_file, mode="w", encoding="utf-8"
        )
        session_formatter = logging.Formatter(
            "%(asctime)s - %(levelname)s - %(message)s", datefmt="%Y-%m-%d %H:%M:%S"
        )
        session_handler.setFormatter(session_formatter)
        self.session_logger.addHandler(session_handler)

        # Console handler for session
        console_handler = logging.StreamHandler()
        console_formatter = logging.Formatter("%(levelname)s: %(message)s")
        console_handler.setFormatter(console_formatter)
        self.session_logger.addHandler(console_handler)

        self.session_logger.info(f"Starting test session: {self.session_name}")
        self.session_logger.info(f"Log directory: {self.base_log_dir}")

    def create_test_capture(self, test_name: str) -> TestLogCapture:
        """Create a new test log capture."""
        if test_name in self.test_captures:
            self.session_logger.warning(f"Test capture already exists for: {test_name}")

        capture = TestLogCapture(test_name, self.base_log_dir)
        self.test_captures[test_name] = capture
        return capture

    def get_test_capture(self, test_name: str) -> Optional[TestLogCapture]:
        """Get an existing test log capture."""
        return self.test_captures.get(test_name)

    @contextmanager
    def capture_test(self, test_name: str):
        """Context manager for capturing a test's logs."""
        capture = self.create_test_capture(test_name)
        capture.start_capture()
        try:
            yield capture
        except Exception as e:
            capture.log_error(f"Test execution failed with exception: {str(e)}")
            capture.end_capture(return_code=1)
            raise
        else:
            capture.end_capture(return_code=0)

    def end_session(self):
        """End the test session."""
        self.session_end_time = datetime.now()
        duration = (self.session_end_time - self.session_start_time).total_seconds()

        # Log session summary
        total_tests = len(self.test_captures)
        passed_tests = sum(
            1 for capture in self.test_captures.values() if capture.return_code == 0
        )
        failed_tests = total_tests - passed_tests

        self.session_logger.info(f"Test session completed: {self.session_name}")
        self.session_logger.info(f"Duration: {duration:.2f} seconds")
        self.session_logger.info(f"Total tests: {total_tests}")
        self.session_logger.info(f"Passed: {passed_tests}")
        self.session_logger.info(f"Failed: {failed_tests}")

        # Create summary file
        self._create_summary_file()

        return self.get_session_summary()

    def _create_summary_file(self):
        """Create a summary file for the test session."""
        summary_file = self.base_log_dir / "summary.json"
        import json

        summary = self.get_session_summary()
        with open(summary_file, "w") as f:
            json.dump(summary, f, indent=2, default=str)

    def get_session_summary(self) -> Dict:
        """Get a summary of the entire test session."""
        duration = 0
        if self.session_end_time:
            duration = (self.session_end_time - self.session_start_time).total_seconds()

        test_summaries = [
            capture.get_summary() for capture in self.test_captures.values()
        ]

        return {
            "session_name": self.session_name,
            "start_time": self.session_start_time.isoformat(),
            "end_time": self.session_end_time.isoformat()
            if self.session_end_time
            else None,
            "duration": duration,
            "total_tests": len(self.test_captures),
            "passed_tests": sum(1 for s in test_summaries if s["status"] == "PASSED"),
            "failed_tests": sum(1 for s in test_summaries if s["status"] == "FAILED"),
            "log_directory": str(self.base_log_dir),
            "tests": test_summaries,
        }

    def create_archive(self, archive_path: Optional[Path] = None) -> Path:
        """Create a zip archive of all test logs."""
        if archive_path is None:
            archive_path = Path.cwd() / f"{self.session_name}_logs.zip"

        self.session_logger.info(f"Creating log archive: {archive_path}")

        with zipfile.ZipFile(archive_path, "w", zipfile.ZIP_DEFLATED) as zipf:
            # Add all files in the log directory
            for file_path in self.base_log_dir.rglob("*"):
                if file_path.is_file():
                    arcname = file_path.relative_to(self.base_log_dir.parent)
                    zipf.write(file_path, arcname)

        archive_size = archive_path.stat().st_size
        self.session_logger.info(
            f"Archive created: {archive_path} ({archive_size} bytes)"
        )

        return archive_path

    def print_session_report(self):
        """Print a detailed session report to console."""
        summary = self.get_session_summary()

        print("\n" + "=" * 80)
        print("TEST SESSION REPORT")
        print("=" * 80)
        print(f"Session: {summary['session_name']}")
        print(f"Duration: {summary['duration']:.2f} seconds")
        print(f"Total Tests: {summary['total_tests']}")
        print(f"Passed: {summary['passed_tests']}")
        print(f"Failed: {summary['failed_tests']}")
        print(f"Log Directory: {summary['log_directory']}")

        if summary["tests"]:
            print("\nTEST DETAILS:")
            print("-" * 80)
            for test in summary["tests"]:
                status_symbol = "✓" if test["status"] == "PASSED" else "✗"
                print(
                    f"{status_symbol} {test['test_name']:<40} {test['duration']:>8.2f}s {test['status']}"
                )

        print("=" * 80)


class ProcessLogCapture:
    """Captures output from a subprocess in real-time."""

    def __init__(self, test_capture: TestLogCapture):
        self.test_capture = test_capture
        self.stdout_thread: Optional[threading.Thread] = None
        self.stderr_thread: Optional[threading.Thread] = None

    def _stream_reader(self, stream: TextIO, log_func):
        """Read from a stream and log each line."""
        try:
            for line in iter(stream.readline, ""):
                if line:
                    log_func(line.rstrip())
        except Exception as e:
            self.test_capture.log_error(f"Error reading stream: {e}")

    def start_capture(self, process: subprocess.Popen):
        """Start capturing output from a process."""
        if process.stdout:
            self.stdout_thread = threading.Thread(
                target=self._stream_reader,
                args=(process.stdout, self.test_capture.log_command_output),
            )
            self.stdout_thread.daemon = True
            self.stdout_thread.start()

        if process.stderr:
            self.stderr_thread = threading.Thread(
                target=self._stream_reader,
                args=(process.stderr, self.test_capture.log_command_error),
            )
            self.stderr_thread.daemon = True
            self.stderr_thread.start()

    def wait_for_completion(self, timeout: Optional[float] = None):
        """Wait for all capture threads to complete."""
        if self.stdout_thread:
            self.stdout_thread.join(timeout)
        if self.stderr_thread:
            self.stderr_thread.join(timeout)


def run_command_with_logging(
    command: List[str],
    test_capture: TestLogCapture,
    timeout: Optional[float] = None,
    cwd: Optional[Path] = None,
    env: Optional[Dict[str, str]] = None,
) -> int:
    """Run a command with comprehensive logging."""
    test_capture.log_info(f"Executing command: {' '.join(command)}")
    if cwd:
        test_capture.log_info(f"Working directory: {cwd}")

    try:
        # Start the process
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            cwd=cwd,
            env=env,
            bufsize=1,
            universal_newlines=True,
        )

        # Start capturing output
        log_capture = ProcessLogCapture(test_capture)
        log_capture.start_capture(process)

        # Wait for completion
        try:
            return_code = process.wait(timeout=timeout)
            log_capture.wait_for_completion(timeout=5)  # Give threads time to finish

            test_capture.log_info(f"Command completed with return code: {return_code}")
            return return_code

        except subprocess.TimeoutExpired:
            test_capture.log_error(f"Command timed out after {timeout} seconds")
            process.kill()
            log_capture.wait_for_completion(timeout=5)
            return -1

    except Exception as e:
        test_capture.log_error(f"Failed to execute command: {e}")
        return -1
