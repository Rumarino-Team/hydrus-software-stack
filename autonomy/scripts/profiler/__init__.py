"""
Profiler Package

This package contains profiling tools for the Hydrus software stack:

- profiling_decorators.py: Function-level profiling decorators and utilities
- ros_profiler.py: Real-time ROS node resource monitoring
- profiling_example.py: Example of how to instrument code with profiling
- profiling_test.py: Simple test without ROS dependencies

Usage:
    from autonomy.scripts.profiler.profiling_decorators import profile_function

    @profile_function("my_function", "category")
    def my_function():
        pass
"""

from .profiling_decorators import (
    FunctionProfiler,
    ProfileManager,
    export_profiling_data,
    profile_function,
    profile_method,
    profile_ros_callback,
)

__all__ = [
    "ProfileManager",
    "FunctionProfiler",
    "profile_function",
    "profile_method",
    "profile_ros_callback",
    "export_profiling_data",
]
