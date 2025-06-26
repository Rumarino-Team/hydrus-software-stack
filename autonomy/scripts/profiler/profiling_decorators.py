#!/usr/bin/env python3
"""
Function Profiling Decorators

Decorators and utilities for profiling function execution times in ROS nodes.
Use these to instrument critical functions for detailed performance analysis.

Usage:
    from autonomy.scripts.services.profiling_decorators import profile_function, ProfileManager

    # Decorate functions you want to profile
    @profile_function("cv_processing")
    def process_image(self, image):
        # Your function code here
        pass

    # Get profiling results
    ProfileManager.get_stats()
"""

import functools
import json
import threading
import time
from collections import defaultdict, deque
from typing import Any, Callable, Dict, List, Optional


class ProfileManager:
    """Global manager for function profiling data"""

    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        self.function_stats = defaultdict(
            lambda: {
                "call_count": 0,
                "total_time": 0.0,
                "min_time": float("inf"),
                "max_time": 0.0,
                "recent_times": deque(maxlen=100),  # Last 100 calls
                "avg_time": 0.0,
                "calls_per_second": 0.0,
                "last_call": 0.0,
            }
        )
        self.node_name = "unknown"
        self._initialized = True

    def set_node_name(self, name: str):
        """Set the name of the current node"""
        self.node_name = name

    def record_function_call(self, function_name: str, execution_time: float):
        """Record a function call and its execution time"""
        with self._lock:
            stats = self.function_stats[function_name]
            current_time = time.time()

            # Update basic stats
            stats["call_count"] += 1
            stats["total_time"] += execution_time
            stats["min_time"] = min(stats["min_time"], execution_time)
            stats["max_time"] = max(stats["max_time"], execution_time)
            stats["recent_times"].append(execution_time)
            stats["last_call"] = current_time

            # Calculate averages
            stats["avg_time"] = stats["total_time"] / stats["call_count"]

            # Calculate calls per second (based on recent calls)
            if len(stats["recent_times"]) > 1:
                time_span = current_time - (current_time - len(stats["recent_times"]))
                if time_span > 0:
                    stats["calls_per_second"] = len(stats["recent_times"]) / time_span

    def get_stats(self) -> Dict[str, Any]:
        """Get current profiling statistics"""
        with self._lock:
            return {"node_name": self.node_name, "functions": dict(self.function_stats)}

    def get_top_functions(
        self, metric: str = "total_time", limit: int = 10
    ) -> List[tuple]:
        """Get top functions by specified metric"""
        with self._lock:
            functions = []
            for func_name, stats in self.function_stats.items():
                if metric in stats:
                    functions.append((func_name, stats[metric]))

            functions.sort(key=lambda x: x[1], reverse=True)
            return functions[:limit]

    def reset_stats(self):
        """Reset all profiling statistics"""
        with self._lock:
            self.function_stats.clear()

    @classmethod
    def get_instance(cls):
        """Get the singleton instance"""
        return cls()


def profile_function(name: Optional[str] = None, category: str = "general"):
    """
    Decorator to profile function execution time

    Args:
        name: Custom name for the function (defaults to function name)
        category: Category for grouping related functions
    """

    def decorator(func: Callable) -> Callable:
        function_name = name or f"{category}.{func.__name__}"

        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            start_time = time.perf_counter()
            try:
                result = func(*args, **kwargs)
                return result
            finally:
                end_time = time.perf_counter()
                execution_time = end_time - start_time
                ProfileManager.get_instance().record_function_call(
                    function_name, execution_time
                )

        return wrapper

    return decorator


def profile_method(name: Optional[str] = None, category: str = "methods"):
    """
    Decorator specifically for class methods

    Args:
        name: Custom name for the method (defaults to class.method)
        category: Category for grouping related methods
    """

    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            if name:
                function_name = name
            else:
                class_name = self.__class__.__name__
                function_name = f"{category}.{class_name}.{func.__name__}"

            start_time = time.perf_counter()
            try:
                result = func(self, *args, **kwargs)
                return result
            finally:
                end_time = time.perf_counter()
                execution_time = end_time - start_time
                ProfileManager.get_instance().record_function_call(
                    function_name, execution_time
                )

        return wrapper

    return decorator


class FunctionProfiler:
    """Context manager for profiling code blocks"""

    def __init__(self, name: str):
        self.name = name
        self.start_time = None

    def __enter__(self):
        self.start_time = time.perf_counter()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.start_time:
            execution_time = time.perf_counter() - self.start_time
            ProfileManager.get_instance().record_function_call(
                self.name, execution_time
            )


def profile_ros_callback(topic_name: str):
    """
    Decorator specifically for ROS callback functions

    Args:
        topic_name: Name of the ROS topic for identification
    """

    def decorator(func: Callable) -> Callable:
        function_name = f"ros_callback.{topic_name}.{func.__name__}"

        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            start_time = time.perf_counter()
            try:
                result = func(*args, **kwargs)
                return result
            finally:
                end_time = time.perf_counter()
                execution_time = end_time - start_time
                ProfileManager.get_instance().record_function_call(
                    function_name, execution_time
                )

        return wrapper

    return decorator


def export_profiling_data(filename: str):
    """Export profiling data to JSON file"""
    stats = ProfileManager.get_instance().get_stats()

    # Convert deque objects to lists for JSON serialization
    for func_stats in stats["functions"].values():
        if "recent_times" in func_stats:
            func_stats["recent_times"] = list(func_stats["recent_times"])

    with open(filename, "w") as f:
        json.dump(stats, f, indent=2, default=str)


# Example usage and testing
if __name__ == "__main__":
    # Example of how to use the profiling decorators

    @profile_function("test_function", "testing")
    def slow_function():
        time.sleep(0.1)
        return "done"

    @profile_method("test_method", "testing")
    class TestClass:
        def slow_method(self):
            time.sleep(0.05)
            return "method done"

    # Test the profiling
    ProfileManager.get_instance().set_node_name("test_node")

    # Call functions multiple times
    for i in range(5):
        slow_function()

    test_obj = TestClass()
    for i in range(3):
        test_obj.slow_method()

    # Use context manager
    with FunctionProfiler("manual_timing"):
        time.sleep(0.02)

    # Print results
    stats = ProfileManager.get_instance().get_stats()
    print("Profiling Results:")
    print(json.dumps(stats, indent=2, default=str))
