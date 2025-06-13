# Hydrus Autonomy System

The Hydrus Autonomy System is a comprehensive ROS-based underwater vehicle control and mission management framework. This README documents all available scripts, their purpose, and usage instructions.

> ⚠️ **Warning**: The mission API is currently under development. Some features may not work as expected and are subject to change.

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Launch Files](#launch-files)
- [Core Components](#core-components)
- [Scripts Reference](#scripts-reference)
- [Mission System Usage](#mission-system-usage)
- [Computer Vision Configuration](#computer-vision-configuration)
- [Hardware Integration](#hardware-integration)

## Overview

The autonomy system provides:
- **Mission Planning**: Hierarchical mission execution for competition tasks
- **Computer Vision**: Object detection using YOLO and color filtering
- **Motion Control**: Precise submarine navigation and thruster control
- **Web Interface**: Real-time monitoring and visualization
- **Serial Communication**: Arduino integration for hardware control

## Architecture

```
autonomy/
├── src/                    # Core Python modules
│   ├── mission_planner/    # Mission execution framework
│   ├── computer_vision/    # CV algorithms and detection
│   ├── controllers.py     # Motion control algorithms
│   ├── cv_publishers.py   # Computer vision ROS publishers
│   └── API.py            # FastAPI web interface
├── scripts/               # Executable scripts
│   ├── controller/        # Control and teleoperation
│   ├── cv/               # Computer vision tools
│   ├── mission/          # Mission management
│   ├── web/              # Web interface
│   └── services/         # ROS services
├── launch/               # ROS launch configurations
├── msg/                  # Custom ROS messages
├── srv/                  # Custom ROS services
├── action/               # ROS actions
└── test/                 # Test files
```

## Launch Files

### Main System Launch
```bash
# Complete autonomy system
roslaunch autonomy simulation.launch

# Mission planner only
roslaunch autonomy mission_planner.launch

# Computer vision publishers
roslaunch autonomy cv_publishers.launch

# Controller system
roslaunch autonomy controller.launch

# Monocular SLAM
roslaunch autonomy monocular_slam.launch
```

## Core Components

### Mission Planner (`src/mission_planner/`)

The mission planner executes hierarchical missions for underwater competition tasks.

**Available Missions:**
- `PreQualificationMission`: Basic movement validation
- `GateMission`: Navigate through gate with style points
- `SlalomMission`: Navigate around buoys in sequence
- `TaggingMission`: Identify and fire torpedoes at targets

**Usage:**
```bash
# Start mission manager
rosrun autonomy mission_manager.py

# Interactive mission control
rosrun autonomy mission_controller.py
```

### Computer Vision (`src/cv_publishers.py`)

Provides object detection using YOLO models and color filtering.

**Topics Published:**
- `/vision/detections`: Object detection results
- `/vision/rgb/image_annotated`: Annotated camera feed

**Services:**
- `/detector/set_color_filter`: Configure color detection
- `/detector/set_yolo_model`: Switch YOLO models

### Motion Control (`src/controllers.py`)

Precise 3-phase movement control (depth → rotation → linear).

**Topics Subscribed:**
- `/zed2i/zed_node/pose`: Current submarine pose

**Action Server:**
- `/navigate_to_waypoint`: Move to target coordinates

### Web Interface (`src/API.py`)

FastAPI-based web interface for monitoring and control.

**Endpoints:**
- `http://localhost:8000/video_feed`: MJPEG camera stream
- `http://localhost:8000/detections/stream`: SSE detection data

## Scripts Reference

### Controller Scripts (`scripts/controller/`)

#### `submarine_teleop.py`
**Purpose**: Manual submarine control via keyboard input
**Usage**:
```bash
rosrun autonomy submarine_teleop.py
```
**Controls**:
- `W/S`: Forward/backward
- `A/D`: Yaw left/right
- `I/K`: Up/down
- `Space`: Stop all motors
- `Q`: Quit

#### `controller_monitor.py`
**Purpose**: Real-time monitoring of controller state and movement phases
**Usage**:
```bash
rosrun autonomy controller_monitor.py
```
**Features**:
- Visual progress indicators
- Phase status (DEPTH, ROTATION, LINEAR)
- Distance calculations
- Completion percentages

#### `thruster_visualizer.py`
**Purpose**: ASCII visualization of thruster states
**Usage**:
```bash
rosrun autonomy thruster_visualizer.py
```
**Features**:
- Color-coded thruster types
- Real-time PWM value display
- Direction indicators (►◄○)

#### `controller_serial.py`
**Purpose**: Direct serial communication with Arduino
**Usage**:
```bash
rosrun autonomy controller_serial.py --port /dev/ttyACM0
```
**Commands**:
- `t1 <pwm>`: Set thruster 1 (1000-2000)
- `d <pwm>`: Set depth motors
- `c <angle>`: Set camera angle (-60 to 60)
- `stop`: Emergency stop

#### `serial_ros_bridge.py`
**Purpose**: Bridge ROS topics to Arduino serial commands
**Usage**:
```bash
rosrun autonomy serial_ros_bridge.py
```
**ROS Parameters**:
- `~port`: Serial port (default: /dev/ttyACM0)
- `~baud_rate`: Baud rate (default: 115200)

### Computer Vision Scripts (`scripts/cv/`)

#### `color_filter_controller.py`
**Purpose**: Interactive color filter configuration
**Usage**:
```bash
# Interactive mode
rosrun autonomy color_filter_controller.py

# Advanced UI mode
rosrun autonomy color_filter_controller.py ui

# Load preset
rosrun autonomy color_filter_controller.py load red
```
**Features**:
- Real-time parameter adjustment
- Color preset management
- YOLO model switching
- Curses-based UI

#### `object_detector.py`
**Purpose**: Standalone object detection testing
**Usage**:
```bash
rosrun autonomy object_detector.py
```

#### `orbs.py`
**Purpose**: ORB feature matching for template tracking
**Usage**:
```bash
rosrun autonomy orbs.py
```
**Controls**:
- `T`: Capture template image
- `Q`: Quit

#### `distance.py`
**Purpose**: Distance estimation using camera
**Usage**:
```bash
rosrun autonomy distance.py
```

### Mission Scripts (`scripts/mission/`)

#### `mission_controller.py`
**Purpose**: Terminal-based mission control interface
**Usage**:
```bash
rosrun autonomy mission_controller.py
```
**Features**:
- Mission selection and management
- Real-time status monitoring
- Progress visualization
- Detailed mission information

**Controls**:
- `↑/↓`: Navigate missions
- `Enter`: Select mission
- `S`: Start mission
- `X`: Stop mission
- `R`: Reset mission
- `D`: View details
- `Q`: Quit

### Web Interface Scripts (`scripts/web/`)

#### `detection_viewer.py`
**Purpose**: Web-based detection visualization
**Usage**:
```bash
rosrun autonomy detection_viewer.py
```
**Access**: http://localhost:5000
**Features**:
- Real-time camera feed
- Bounding box overlays
- Color-coded detectors

### Testing Scripts

#### `arduino_simulator.py`
**Purpose**: Simulate submarine movement for controller testing
**Usage**:
```bash
# Default 10-second test
rosrun autonomy arduino_simulator.py

# Custom duration
rosrun autonomy arduino_simulator.py 15.0
```
**Features**:
- Realistic movement simulation
- Thruster command validation
- Real-time position updates

#### `test_controller.py`
**Purpose**: Unit tests for controller components
**Usage**:
```bash
rosrun autonomy test_controller.py
```

## Mission System Usage

### Starting a Mission

1. **Launch the system**:
   ```bash
   roslaunch autonomy simulation.launch
   ```

2. **Start mission manager**:
   ```bash
   rosrun autonomy mission_controller.py
   ```

3. **Select and start mission**:
   - Use arrow keys to select mission
   - Press `Enter` to activate
   - Press `S` to start execution

### Mission Types

#### Gate Mission
Navigate through an underwater gate with style points for rotations.
- Searches for gate and animal markers
- Chooses optimal side based on animal selection
- Performs stylistic maneuvers while passing through

#### Slalom Mission
Navigate around buoys in a specific sequence.
- Follows waypoint-based search pattern
- Identifies buoy colors and positions
- Executes slalom navigation

#### Tagging Mission
Identify targets and fire torpedoes.
- Searches for target objects
- Approaches and identifies targets
- Fires torpedoes at correct targets

### Custom Missions

To create a new mission:

1. **Create mission class**:
   ```python
   # src/mission_planner/my_mission.py
   from mission_planner.base_mission import BaseMission

   class MyMission(BaseMission):
       def __init__(self):
           super().__init__("my_mission")
           self.mission_tree_root = self.build_mission_tree()
   ```

2. **Register in mission manager**:
   ```python
   # In mission_manager.py
   self.register_mission("my_mission", MyMission())
   ```

## Computer Vision Configuration

### YOLO Models

Place YOLO model files (`.pt`) in `/yolo_models/` directory:
```bash
# Switch models via service
rosservice call /detector/set_yolo_model "model_name: 'yolov8n.pt'"

# Or use interactive controller
rosrun autonomy color_filter_controller.py ui
```

### Color Filtering

Configure color detection parameters:
```bash
# Interactive configuration
rosrun autonomy color_filter_controller.py

# Load preset
rosrun autonomy color_filter_controller.py load red
```

**Color Presets** (stored in `scripts/cv/color_presets/`):
- `red.json`: Red object detection
- `green.json`: Green object detection
- `blue.json`: Blue object detection
- `yellow.json`: Yellow object detection

## Hardware Integration

### Arduino Communication

The system communicates with Arduino via serial for thruster control:

**Serial Command Format**:
- `T1:1500` - Set thruster 1 to PWM 1500 (neutral)
- `D:1600` - Set depth motors to PWM 1600 (up)
- `P:1400` - Set torpedo motors to PWM 1400 (reverse)
- `C:30` - Set camera to 30° angle

**PWM Range**: 1000-2000 (1500 = neutral)

### Thruster Layout

```
T1 ---- T5
|        |
T2      T6
|        |
T3 ---- T7
|        |
T4 ---- T8
```

- **T1, T4, T5, T8**: Regular thrusters (movement/rotation)
- **T2, T7**: Torpedo thrusters
- **T3, T6**: Depth thrusters

---

**Last Updated**: June 2025
**ROS Version**: Noetic
**Python Version**: 3.8+
