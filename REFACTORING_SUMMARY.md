# Hydrus Software Stack Refactoring Summary

## Overview

The Hydrus software stack has been refactored to create a clear separation of concerns between Docker container management and Hydrus software control. This refactoring introduces `hydrus-cli` as the central intermediate layer for all Hydrus software operations.

## New Architecture

### 1. **hocker** - Pure Docker Management
- **Focus**: Docker containers, volumes, networking, and deployment
- **Responsibility**: Platform detection, container lifecycle, compose file management
- **No longer handles**: Hydrus software operations, ROS commands, or application logic

### 2. **hydrus-cli** - Central Hydrus Software Controller
- **Focus**: All Hydrus software operations (ROS, Arduino, testing, monitoring, etc.)
- **Responsibility**: The intermediate layer that controls all aspects of the Hydrus software stack
- **Capabilities**:
  - Workspace building and testing
  - Hardware control (Arduino compilation, serial communication)
  - Data management (rosbag download, playback, conversion)
  - Monitoring and visualization (tmux, RViz, web UI, profiling)
  - Development tools (formatting, linting)

### 3. **ros-entrypoint** - Simple Delegation
- **Focus**: Pure delegation to hydrus-cli
- **Responsibility**: Parse Docker environment variables and delegate to hydrus-cli
- **No longer has**: Legacy fallback modes or direct ROS operations

## New Workflow

### Starting Containers
```bash
# Start development environment
./docker/hydrus-docker/hocker --dev

# Start in detached mode
./docker/hydrus-docker/hocker --dev --detach

# Enter running container
./docker/hydrus-docker/hocker --exec
```

### Inside Container - Using hydrus-cli
```bash
# Basic operations
hydrus-cli build                    # Build workspace
hydrus-cli test                     # Run tests
hydrus-cli --help                   # Show all options

# Development workflow
hydrus-cli build --tmux --arduino-compile --virtual-arduino
hydrus-cli tmux --monitor --profile

# Simulation workflow
hydrus-cli rosbag-download --rosbag-play --rviz --web-ui

# Testing workflow
hydrus-cli test --rosbag-download

# Hardware deployment
hydrus-cli build --arduino-compile --serial-bridge --tmux

# Use predefined configurations
hydrus-cli --config development     # Equivalent to build + tmux + hardware setup
hydrus-cli --config simulation      # Equivalent to rosbag + rviz + web-ui
hydrus-cli --config test            # Equivalent to test + rosbag-download
hydrus-cli --config production      # Equivalent to build + arduino + monitoring
```

## Available hydrus-cli Commands

### Primary Commands
- `build` - Build the ROS workspace
- `test` - Run the complete test suite
- `tmux` - Start tmux monitoring sessions
- `monitor` - Start monitoring and debugging tools

### Hardware Control
- `arduino-compile` - Compile and upload Arduino code
- `virtual-arduino` - Start virtual Arduino processes
- `serial-bridge` - Start serial ROS bridge

### Data & Simulation
- `rosbag-download` - Download rosbag files
- `rosbag-play` - Start rosbag playback
- `rosbag-convert` - Convert video to rosbag

### Visualization & UI
- `rviz` - Start RViz visualization
- `web-ui` - Start web user interface
- `api-server` - Start API server

### Development Tools
- `format` - Format code with project standards
- `lint` - Run code quality checks
- `profile` - Start ROS node profiler

### Flags
- `--no-build` - Skip workspace build
- `--clean` - Clean workspace before building
- `--tmux` - Start tmux sessions
- `--monitor` - Start monitoring tools
- `--arduino-compile` - Compile Arduino
- `--virtual-arduino` - Start virtual Arduino
- `--serial-bridge` - Start serial bridge
- `--rosbag-download` - Download rosbags
- `--rosbag-play` - Start rosbag playback
- `--rosbag-loop` - Loop rosbag playback
- `--rviz` - Start RViz
- `--web-ui` - Start web interface
- `--api-server` - Start API server
- `--format` - Format code
- `--lint` - Run linting
- `--profile` - Start profiler

### Predefined Configurations
- `--config test` - Testing configuration
- `--config development` - Development with hardware
- `--config simulation` - Simulation with rosbags and visualization
- `--config production` - Production deployment
- `--config competition` - Competition-ready setup

## Benefits of Refactoring

### 1. **Clear Separation of Concerns**
- Docker management is isolated from software operations
- Each component has a single, well-defined responsibility
- Easier to maintain and extend

### 2. **Simplified Workflow**
- Single command interface (`hydrus-cli`) for all software operations
- Consistent command structure and help system
- Predefined configurations for common use cases

### 3. **Better Error Handling**
- Clear error messages and helpful guidance
- No confusing fallback modes
- Proper exit codes for automation

### 4. **Enhanced Functionality**
- More comprehensive command options
- Better integration between different components
- Support for development workflows and tools

### 5. **Improved Documentation**
- Self-documenting command structure
- Comprehensive help system
- Clear examples for common workflows

## Migration Guide

### For Users
- **Old**: `hocker --test` then manual commands inside container
- **New**: `hocker --test` (container starts) → `hocker --exec` (enter container) → `hydrus-cli test`

### For Developers
- **Old**: Call scripts directly (start_tmux_sessions.py, download_rosbag.py, etc.)
- **New**: Use `hydrus-cli` commands that wrap and coordinate these scripts

### For CI/CD
- **Old**: `hocker --test` with environment variables
- **New**: Same workflow, but cleaner delegation to `hydrus-cli`

## Implementation Details

### Files Modified
- `docker/hydrus-docker/hydrus_cli.py` - Enhanced with comprehensive functionality
- `docker/hydrus-docker/hocker` - Simplified to pure Docker management
- `scripts/ros-entrypoint.py` - Simplified to pure delegation
- `scripts/ros-entrypoint-new.py` - Created clean delegation version

### Files Preserved
- All existing scripts (start_tmux_sessions.py, download_rosbag.py, etc.) are unchanged
- hydrus-cli calls these scripts rather than replacing them
- Backward compatibility is maintained for direct script usage

### New Features in hydrus-cli
- Comprehensive argument parser with detailed help
- Support for combining multiple operations in single command
- Predefined configuration profiles
- Better error handling and user feedback
- Integration with all existing Hydrus scripts and tools

## Testing the New System

1. **Build containers**: `./docker/hydrus-docker/hocker --dev --detach`
2. **Enter container**: `./docker/hydrus-docker/hocker --exec`
3. **Test hydrus-cli**: `hydrus-cli --help`
4. **Run development setup**: `hydrus-cli --config development`
5. **Run tests**: `hydrus-cli test --rosbag-download`

The refactoring maintains full backward compatibility while providing a much cleaner and more maintainable architecture for the Hydrus software stack.
