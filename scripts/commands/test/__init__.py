"""
Test command module for Hydrus software stack.

This module contains all test-related functionality including:
- Test execution and management
- Test configuration loading from distributed .hss files
- Test logging and result archiving
- Test caching functionality
"""

from .test import test_app

__all__ = ["test_app"]
