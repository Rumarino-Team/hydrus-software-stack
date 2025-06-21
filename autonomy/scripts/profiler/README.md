# Profiler Package

This directory contains all profiling-related tools for the Hydrus software stack.

## Files

### Core Profiling Tools
- **`profiling_decorators.py`** - Function-level profiling decorators and utilities
- **`ros_profiler.py`** - Real-time ROS node resource monitoring tool
- **`__init__.py`** - Package initialization with imports

### Examples and Tests
- **`profiling_example.py`** - Complete example of instrumenting ROS CV code
- **`profiling_test.py`** - Standalone test without ROS dependencies

## Quick Start

### 1. Simple Test (No ROS Required)
```bash
python3 autonomy/scripts/profiler/profiling_test.py
```

### 2. Instrument Your Code
```python
from autonomy.scripts.profiler.profiling_decorators import profile_function

@profile_function("my_function", "category")
def my_function():
    # Your code here
    pass
```

### 3. Monitor ROS Nodes
```bash
# Monitor all nodes
python3 autonomy/scripts/profiler/ros_profiler.py

# Monitor specific nodes with export
python3 autonomy/scripts/profiler/ros_profiler.py --nodes cv_publisher --export data.csv
```

## Documentation

See `docs/PROFILING.md` for comprehensive usage guide.

## Features

- **Function-level timing** with decorators and context managers
- **Real-time ROS node monitoring** (CPU, memory, FPS)
- **Export capabilities** (JSON, CSV)
- **Category-based organization** of profiled functions
- **Statistical analysis** (min, max, average, recent history)
- **Background process tracking** for ROS nodes
- **Topic publishing rate monitoring**
