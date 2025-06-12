# Docker Setup and Configuration

This directory contains Docker configurations for deploying the Hydrus Software Stack across different platforms and environments. Docker ensures consistent deployment regardless of your host system configuration.

## Why Docker?

Docker is our primary deployment technology for several critical reasons:

- **Python Dependency Management**: Eliminates "Dependency Hell" by containerizing all Python libraries and versions
- **ROS Version Consistency**: Manages different ROS distributions and prevents version conflicts
- **Simplified Deployment**: Automates configuration and eliminates manual setup steps
- **Hardware Abstraction**: Supports different architectures (AMD64, ARM64, Jetson) and GPU configurations
- **Environment Isolation**: Prevents conflicts between development and production environments

## Prerequisites

### All Platforms
- [Docker](https://www.docker.com/) installed and running
- Docker Compose (included with Docker Desktop)

### Linux/WSL
```bash
# Verify Docker is working
docker --version
docker compose --version
```

### Windows
- Windows Subsystem for Linux (WSL) 2
- Docker Desktop with WSL integration enabled

### NVIDIA GPU Support (Optional)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- CUDA drivers installed on host system

## Quick Start

### Basic Usage
```bash
# Make script executable
chmod +x docker/run_docker.sh

# Run with automatic platform detection
./docker/run_docker.sh

# Test installation
./docker/run_docker.sh --test
```

## Available Configurations

### Platform Detection
The script automatically detects your platform and selects the appropriate configuration:

| Platform | Configuration | Description |
|----------|---------------|-------------|
| **WSL** | `docker-compose-amd64-cpu.yaml` | Windows Subsystem for Linux |
| **Jetson TX2** | Jetson scripts | ARM64 embedded platform |
| **NVIDIA GPU** | `docker-compose-amd64-cuda.yaml` | GPU-accelerated processing |
| **CPU Only** | `docker-compose-amd64-cpu.yaml` | Standard x86_64 systems |

### Docker Compose Files

#### `docker-compose-amd64-cpu.yaml`
- **Use Case**: Development, testing, CPU-only systems
- **Services**: ROS Master, Hydrus CPU container
- **Features**: Basic autonomy stack without GPU acceleration

#### `docker-compose-amd64-cuda.yaml`
- **Use Case**: Production, GPU-accelerated computer vision
- **Services**: ROS Master, ZED Camera, Hydrus CUDA container
- **Features**: Full GPU support, ZED camera integration, hardware acceleration

#### `docker-compose-jetson-tx2.yaml`
- **Use Case**: Embedded deployment on Jetson platforms
- **Services**: Optimized for ARM64 architecture
- **Features**: Low-power operation, embedded-specific optimizations

## Command Line Options

### Core Options

#### `--test`
**Purpose**: Automated testing mode
```bash
./docker/run_docker.sh --test
```
- Forces CPU-only mode
- Disables interactive prompts
- Uses `--abort-on-container-exit` for CI/CD
- Skips RViz and visualization components

#### `--force-cpu`
**Purpose**: Force CPU-only deployment
```bash
./docker/run_docker.sh --force-cpu
```
- Overrides GPU detection
- Uses CPU-only Docker Compose
- Useful for debugging or resource-constrained environments

#### `--force-jetson`
**Purpose**: Force Jetson deployment
```bash
./docker/run_docker.sh --force-jetson
```
- Bypasses platform detection
- Runs Jetson-specific configuration
- Executes `../jetson.sh` script

#### `--deploy`
**Purpose**: Enable hardware deployment mode
```bash
./docker/run_docker.sh --deploy
```
- Configures Arduino upload functionality
- Enables hardware integration features
- Sets deployment-specific environment variables

#### `--volume`
**Purpose**: Enable volume mounting for development
```bash
./docker/run_docker.sh --volume
```
- Mounts additional development volumes
- Enables live code editing without rebuilds
- Useful for active development workflows

### Option Combinations
```bash
# Testing with hardware deployment
./docker/run_docker.sh --test --deploy

# Development with CPU override
./docker/run_docker.sh --force-cpu --volume

# All options combined
./docker/run_docker.sh --deploy --volume --force-cpu
```

## Interactive Features

When running without `--test` mode, the script prompts for additional configuration:

### ROS Bag Playback
```
Do you want to play rosbag files from the rosbags folder? (y/n)
```
- **Yes**: Enables automatic rosbag playback from `/rosbags` directory
- **Auto-download**: Offers to download default rosbag if none exist
- **Uses**: `download_rosbag.py` for automated downloading

### Arduino Debug Monitoring
```
Do you want to monitor Arduino serial output for debugging? (y/n)
```
- **Purpose**: Real-time serial port monitoring
- **Shows**: Arduino logs before ROS node initialization
- **Useful**: Hardware debugging and development

### RViz Visualization (CPU/WSL only)
```
Do you want to use RViz? (y/n)
```
- **Available**: CPU mode and WSL environments
- **Purpose**: 3D visualization of robot state and sensor data
- **Requires**: X11 forwarding for WSL

### ZED Camera Display (GPU mode only)
```
Do you want to display the ZED2i camera in RViz? (y/n)
```
- **Available**: NVIDIA GPU configurations only
- **Purpose**: Real-time camera feed visualization
- **Performance**: GPU-accelerated image processing

## Environment Variables

The script sets various environment variables for container configuration:

| Variable | Purpose | Values |
|----------|---------|---------|
| `DEPLOY` | Hardware deployment mode | `true`/`false` |
| `VOLUME` | Development volume mounting | `true`/`false` |
| `RVIZ` | Enable RViz visualization | `true`/`false` |
| `ZED_OPTION` | ZED camera in RViz | `true`/`false` |
| `ROSBAG_PLAYBACK` | Automatic rosbag playback | `true`/`false` |
| `DEBUG_ARDUINO` | Arduino serial monitoring | `true`/`false` |
| `TEST` | Automated testing mode | `true`/`false` |

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

## Development Workflow

### Local Development
```bash
# Start development environment
./docker/run_docker.sh --volume

# Make code changes on host
# Changes are immediately available in container
```

### Testing Pipeline
```bash
# Automated testing
./docker/run_docker.sh --test

# Integration testing with hardware
./docker/run_docker.sh --test --deploy
```

### Hardware Deployment
```bash
# Production deployment
./docker/run_docker.sh --deploy

# With debug monitoring
./docker/run_docker.sh --deploy
# Select 'y' for Arduino monitoring
```


### Debug Commands

#### List Running Containers
```bash
docker ps
```

#### Enter Container Shell
```bash
# Find container name/ID
docker ps

# Enter container
docker exec -it <container_name> bash
```

#### View Container Logs
```bash
# All services
docker compose logs

# Specific service
docker compose logs hydrus_cpu
```


For general Hydrus issues, see the main [README](../README.md) and [autonomy documentation](../autonomy/README.md).


