# Function and ROS Node Profiling Guide

This document explains how to instrument and profile your Hydrus software using the provided profiling decorators and ROS node profiler.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Profiling Decorators](#profiling-decorators)
  - [Installation](#installation)
  - [Decorator Usage](#decorator-usage)
  - [Context Manager Usage](#context-manager-usage)
  - [Example: `profiling_decorators.py`](#example-profiling_decoratorspy)
- [Instrumenting Your Code](#instrumenting-your-code)
  - [Python Functions](#python-functions)
  - [Class Methods](#class-methods)
  - [ROS Callbacks](#ros-callbacks)
- [Example Profiling Script](#example-profiling-script)
- [ROS Node Profiler](#ros-node-profiler)
  - [Usage](#usage)
  - [CLI Options](#cli-options)
- [Exporting and Analyzing Results](#exporting-and-analyzing-results)
- [Best Practices](#best-practices)

---

## Overview

Hydrus provides two complementary profiling tools:

1. **Function-level Decorators**: Lightweight decorators and context managers for measuring execution time of individual functions or code blocks.
2. **ROS Node Profiler**: A standalone ROS node (`ros_profiler.py`) for real-time resource monitoring and FPS measurement of active ROS nodes.

---

## Prerequisites

- Python 3.8+ environment
- ROS (Noetic/Melodic) installed and sourced in your environment
- `psutil` and `colorama` Python packages installed for the ROS profiler:

```bash
pip install psutil colorama
```

**Important**: Make sure to source your ROS environment before running any profiling tools:

```bash
# Source ROS
source /opt/ros/noetic/setup.bash  # or melodic

# Source your workspace (if built)
source devel/setup.bash
```

---

## Profiling Decorators

### Installation

The decorators and utilities live in `autonomy/scripts/profiler/profiling_decorators.py`. No additional install steps are needed if your ROS package path includes this folder.

### Decorator Usage

- `@profile_function(name?: str, category: str="general")`
- `@profile_method(name?: str, category: str="methods")`
- `@profile_ros_callback(topic_name: str)`

Apply these directly above the function or method definition:

```python
from autonomy.scripts.profiler.profiling_decorators import profile_function, profile_method, profile_ros_callback

@profile_function("cv.yolo_inference", "cv")
def yolo_object_detection(image):
    # ...
    pass

@profile_method(category="mission")
def run(self):
    # ...
    pass

@profile_ros_callback("camera/image_raw")
def image_callback(msg):
    # ...
    pass
```

### Context Manager Usage

Use `FunctionProfiler(name: str)` to wrap arbitrary code blocks:

```python
from autonomy.scripts.profiler.profiling_decorators import FunctionProfiler

with FunctionProfiler("cv.contour_detection"):
    contours, _ = cv2.findContours(mask, ...)
```

### Example: `profiling_decorators.py`

See `autonomy/scripts/profiler/profiling_decorators.py` for full implementation details and helper functions:

- `ProfileManager` singleton collects statistics
- `export_profiling_data(filename)` to dump JSON

---

## Instrumenting Your Code

### Python Functions

Decorate any standalone function:

```python
@profile_function("depth.calculation")
def calculate_depth(points, intrinsics):
    # core logic
    return points_3d
```

### Class Methods

Decorate methods inside your classes:

```python
class MissionPlanner:

    @profile_method(category="mission")
    def feedback_callback(self, feedback):
        # process feedback
        pass
```

### ROS Callbacks

Wrap ROS subscriber callbacks:

```python
@profile_ros_callback("/detector/box_detection")
def detection_callback(msg):
    # parse detections
    pass
```

---

## Example Profiling Script

An example instrumentation is provided in `autonomy/scripts/profiler/profiling_example.py`. To run:

```bash
# First, ensure ROS is sourced
source /opt/ros/noetic/setup.bash  # or melodic
source devel/setup.bash           # if workspace is built

# Method 1: Direct execution
python3 autonomy/scripts/profiler/profiling_example.py

# Method 2: Using rosrun (after making executable)
chmod +x autonomy/scripts/profiler/profiling_example.py
rosrun autonomy profiling_example.py
```

**Note**: The example script requires ROS to be properly sourced and `rospy` to be available.

### Simple Profiling Test (No ROS Required)

A standalone test without ROS dependencies is available in `autonomy/scripts/profiler/profiling_test.py`:

```bash
# Run the simple profiling test
python3 autonomy/scripts/profiler/profiling_test.py

# This will:
# - Test all profiling decorators
# - Generate mock data and run profiled functions
# - Display performance results
# - Export detailed JSON results
```

This test is useful for:
- Verifying profiling functionality without ROS
- Understanding profiling output format
- Testing on systems without ROS installed

Inside the example script, you will see usage of:
- `ProfileManager.get_instance().set_node_name("cv_publisher")`
- Decorators and `FunctionProfiler` blocks
- Automatic export via `atexit` to `cv_publisher_profile.json`

---

## ROS Node Profiler

The ROS Node Profiler (`autonomy/scripts/profiler/ros_profiler.py`) monitors:
- CPU & memory usage per node
- Topic publish rates (FPS)
- Optional function-level profiling if your nodes use the decorators

### Usage

1. Compile and source your workspace:

    ```bash
    # Build the workspace
    catkin_make
    source devel/setup.bash
    ```

2. Run the profiler with:

    ```bash
    # Method 1: Direct execution
    python3 autonomy/scripts/profiler/ros_profiler.py [options]

    # Method 2: Using rosrun (after making executable)
    chmod +x autonomy/scripts/profiler/ros_profiler.py
    rosrun autonomy ros_profiler.py [options]
    ```

3. View real-time console dashboard.

**Example commands:**

```bash
# Monitor all nodes with default settings
python3 autonomy/scripts/profiler/ros_profiler.py

# Monitor specific nodes only
python3 autonomy/scripts/profiler/ros_profiler.py --nodes cv_publisher controller_action

# Enable function-level profiling and export data
python3 autonomy/scripts/profiler/ros_profiler.py --profile-functions --export profile_data.csv

# Custom update rate
python3 autonomy/scripts/profiler/ros_profiler.py --update-rate 1.0
```

### CLI Options

- `--nodes`: List of specific node names to monitor (default = all)
- `--profile-functions`: Include function-level stats (requires your nodes to use decorators)
- `--export`: CSV filename to record data over time
- `--update-rate`: Sampling interval in seconds (default = 0.5s)

---

## Exporting and Analyzing Results

- **Function Decorator Exports**: Use `export_profiling_data("output.json")` at shutdown to dump collected JSON.
- **ROS Node Profiler Exports**: CSV export via `--export` and summary JSON created alongside.

Imported JSON can be visualized with tools like Excel, Python scripts, or custom dashboards.

---

## Best Practices

- Profile only critical or slow code paths to reduce overhead.
- Use categories and clear naming conventions for easy filtering.
- Combine real-time `ros_profiler` with offline JSON analysis for comprehensive insight.

## Troubleshooting

### Common Issues

**1. "ModuleNotFoundError: No module named 'rospy'"**
```bash
# Solution: Source your ROS environment
source /opt/ros/noetic/setup.bash  # or melodic
source devel/setup.bash           # if workspace is built
```

**2. "rosrun: Couldn't find executable"**
```bash
# Solution: Make script executable and ensure it's in the right location
chmod +x autonomy/scripts/profiler/profiling_example.py
# Or run directly with python3
python3 autonomy/scripts/profiler/profiling_example.py
```

**3. "package 'autonomy' not found"**
```bash
# Solution: Build and source your workspace
catkin_make
source devel/setup.bash
```

**4. Missing dependencies for ros_profiler.py**
```bash
# Solution: Install required Python packages
pip install psutil colorama
```

---

_This guide helps you leverage detailed profiling across your Hydrus software stack for performance tuning and diagnostics._
