# Hocker - Hydrus Docker Deployment Tool

**Hocker** (Hydrus Docker) is our intelligent deployment orchestrator that automatically detects your platform and configures the optimal Docker environment for the Hydrus Software Stack. No more guessing which Docker Compose file to use or manually setting environment variables - Hocker handles it all.

## Why Hocker?

Traditional Docker deployment required manual configuration selection and environment variable management. Hocker eliminates this complexity by:

- **Automatic Platform Detection**: Detects CPU, NVIDIA GPU, WSL, or Jetson platforms
- **Intelligent Configuration**: Applies optimal settings based on your hardware and use case
- **Configuration Groups**: Pre-defined settings for development, testing, competition, and simulation
- **Modular Architecture**: Clean separation of concerns with dedicated managers for different aspects
- **Environment Management**: Handles all Docker Compose files and environment variables automatically

## Quick Start

### Make Hocker Executable
```bash
chmod +x docker/hydrus-docker/hocker
```

### Basic Usage
```bash
# Run with automatic platform detection
./docker/hydrus-docker/hocker

# Test your installation
./docker/hydrus-docker/hocker --test

# Development mode with live code editing
./docker/hydrus-docker/hocker --dev
```

## Configuration Groups

Hocker provides pre-configured groups that bundle common settings:

### `--dev` (Development)
```bash
./docker/hydrus-docker/hocker --dev
```
- **Purpose**: Active development with live code editing
- **Features**: Volume mounting, RViz enabled, development tools
- **Best for**: Writing and testing code changes

### `--test` (Automated Testing)
```bash
./docker/hydrus-docker/hocker --test
```
- **Purpose**: CI/CD and automated testing pipelines
- **Features**: CPU-only, no interactivity, abort on container exit
- **Best for**: GitHub Actions, automated validation

### `--competition` (Competition Deployment)
```bash
./docker/hydrus-docker/hocker --competition
```
- **Purpose**: RobSub competition environment
- **Features**: Hardware deployment, optimized performance
- **Best for**: Competition day, performance testing

### `--simulation` (Simulation Mode)
```bash
./docker/hydrus-docker/hocker --simulation
```
- **Purpose**: Simulation with rosbag playback
- **Features**: Automatic rosbag download, playback enabled
- **Best for**: Algorithm testing, mission replay

## Platform Detection

Hocker automatically detects your platform and selects the appropriate Docker configuration:

| Platform | Auto-Selected Configuration | Description |
|----------|----------------------------|-------------|
| **WSL** | `docker-compose-amd64-cpu.yaml` | Windows Subsystem for Linux |
| **Jetson TX2** | Jetson deployment scripts | ARM64 embedded platform |
| **NVIDIA GPU** | `docker-compose-amd64-cuda.yaml` | GPU-accelerated processing |
| **CPU Only** | `docker-compose-amd64-cpu.yaml` | Standard x86_64 systems |

## Advanced Options

### Force Platform Override
```bash
# Force CPU-only (override GPU detection)
./docker/hydrus-docker/hocker --force-cpu

# Force Jetson deployment
./docker/hydrus-docker/hocker --force-jetson
```

### Individual Features
```bash
# Enable hardware deployment mode
./docker/hydrus-docker/hocker --deploy

# Enable volume mounting for development
./docker/hydrus-docker/hocker --volume

# Enable RViz visualization
./docker/hydrus-docker/hocker --rviz

# Enable rosbag playback
./docker/hydrus-docker/hocker --rosbag-playback
```

### Container Management
```bash
# Execute into running container
./docker/hydrus-docker/hocker --exec

# Destroy all containers and cleanup
./docker/hydrus-docker/hocker --destroy
```

### Option Combinations
```bash
# Development with hardware integration
./docker/hydrus-docker/hocker --dev --deploy

# Testing with CPU override
./docker/hydrus-docker/hocker --test --force-cpu

# Custom configuration
./docker/hydrus-docker/hocker --volume --rviz --rosbag-playback
```

## Architecture Overview

Hocker is built with a modular architecture:

```
hocker (main script)
├── cli.py              # Command line argument parsing
├── config.py           # Configuration management and groups
├── system_detector.py  # Platform and hardware detection
├── docker_manager.py   # Docker Compose orchestration
└── vscode_integration.py # VS Code development integration
```

### Why Docker?

Docker is our primary deployment technology for several critical reasons:

- **Python Dependency Management**: Eliminates "Dependency Hell" by containerizing all Python libraries and versions
- **ROS Version Consistency**: Manages different ROS distributions and prevents version conflicts  
- **Simplified Deployment**: Automates configuration and eliminates manual setup steps
- **Hardware Abstraction**: Supports different architectures (AMD64, ARM64, Jetson) and GPU configurations
- **Environment Isolation**: Prevents conflicts between development and production environments

## Docker Compose Files

### `docker-compose-amd64-cpu.yaml`
- **Use Case**: Development, testing, CPU-only systems
- **Services**: ROS Master, Hydrus CPU container
- **Features**: Basic autonomy stack without GPU acceleration

### `docker-compose-amd64-cuda.yaml`
- **Use Case**: Production, GPU-accelerated computer vision
- **Services**: ROS Master, ZED Camera, Hydrus CUDA container
- **Features**: Full GPU support, ZED camera integration, hardware acceleration

### `docker-compose-jetson-tx2.yaml`
- **Use Case**: Embedded deployment on Jetson platforms
- **Services**: Optimized for ARM64 architecture
- **Features**: Low-power operation, embedded-specific optimizations

## Environment Variables

Hocker automatically sets these environment variables based on configuration:

| Variable | Purpose | Values |
|----------|---------|---------|
| `DEPLOY` | Hardware deployment mode | `true`/`false` |
| `VOLUME` | Development volume mounting | `true`/`false` |
| `RVIZ` | Enable RViz visualization | `true`/`false` |
| `ZED_OPTION` | ZED camera in RViz | `true`/`false` |
| `ROSBAG_PLAYBACK` | Automatic rosbag playback | `true`/`false` |
| `DEBUG_ARDUINO` | Arduino serial monitoring | `true`/`false` |
| `TEST` | Automated testing mode | `true`/`false` |
| `FORCE_CPU` | Force CPU-only mode | `true`/`false` |
| `VSCODE` | VS Code integration | `true`/`false` |

## Interactive Features

When running without `--test` mode, Hocker provides interactive prompts for additional configuration:

### Automatic Rosbag Download
```
No rosbag files found. Download sample rosbag? (y/n)
```
- **Purpose**: Automatically downloads sample data for simulation
- **Uses**: `download_rosbag.py` for automated downloading

### Arduino Debug Monitoring
```
Enable Arduino serial output monitoring? (y/n)
```
- **Purpose**: Real-time serial port monitoring
- **Shows**: Arduino logs before ROS node initialization
- **Useful**: Hardware debugging and development

### RViz Visualization
```
Enable RViz for 3D visualization? (y/n)
```
- **Available**: CPU mode and WSL environments
- **Purpose**: 3D visualization of robot state and sensor data
- **Requires**: X11 forwarding for WSL

### ZED Camera Display
```
Display ZED2i camera feed in RViz? (y/n)
```
- **Available**: NVIDIA GPU configurations only
- **Purpose**: Real-time camera feed visualization
- **Performance**: GPU-accelerated image processing

## Development Workflow

### Local Development
```bash
# Start development environment
./docker/hydrus-docker/hocker --dev

# Make code changes on host
# Changes are immediately available in container
```

### Testing Pipeline
```bash
# Automated testing
./docker/hydrus-docker/hocker --test

# Integration testing with hardware
./docker/hydrus-docker/hocker --test --deploy
```

### Hardware Deployment
```bash
# Competition deployment
./docker/hydrus-docker/hocker --competition

# With custom settings
./docker/hydrus-docker/hocker --deploy --rviz
```

## Volumes and Ports

### Mounted Volumes
- **Source Code**: `../:/home/catkin_ws/src/hydrus-software-stack`
- **ROS Bags**: `../rosbags:/rosbags`
- **YOLO Models**: `../yolo_models:/yolo_models`
- **X11 Socket**: `/tmp/.X11-unix:/tmp/.X11-unix` (for GUI applications)

### Exposed Ports
- **8000**: FastAPI web interface
- **5000**: Flask detection viewer
- **11311**: ROS Master

### Device Access
- **Serial Ports**: `/dev/ttyACM0`, `/dev/ttyACM1`, `/dev/ttyUSB0`
- **Graphics**: `/dev/dri` (for hardware acceleration)
- **GPU**: NVIDIA devices (CUDA configurations only)

## Troubleshooting

### Debug Commands

#### Check Running Containers
```bash
docker ps
```

#### Enter Container Shell
```bash
./docker/hydrus-docker/hocker --exec
```

#### View Container Logs
```bash
# All services
docker compose logs

# Specific service
docker compose logs hydrus_cpu
```

#### Clean Reset
```bash
# Destroy all containers and start fresh
./docker/hydrus-docker/hocker --destroy
./docker/hydrus-docker/hocker --dev
```

### Common Issues

#### Permission Errors
```bash
# Make sure hocker is executable
chmod +x docker/hydrus-docker/hocker
```

#### Docker Not Running
```bash
# Start Docker service
sudo systemctl start docker

# Or start Docker Desktop on Windows/Mac
```

#### GPU Not Detected
```bash
# Force CPU mode for testing
./docker/hydrus-docker/hocker --force-cpu

# Check NVIDIA Docker setup
nvidia-smi
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

For general Hydrus issues, see the main [README](../README.md) and [autonomy documentation](../autonomy/README.md).


