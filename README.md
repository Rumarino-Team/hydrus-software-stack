# Hydrus Software Stack

The Hydrus Software Stack is a comprehensive ROS-based toolkit for autonomous underwater vehicles (AUVs) and RobSub competitions. Built with maintainability, usability, and excellent software engineering practices at its core.

## 📖 Documentation

- **[Philosophy & Contributions](PHILOSOPHY.md)** - **Read this first!** Our development principles and contribution guidelines
- **[Autonomy System](autonomy/README.md)** - Complete guide to all autonomy scripts and components
- **[Docker Setup](docker/README.md)** - Detailed Docker configuration and deployment options

## 🚀 Quick Start

### Clone the Repository
```bash
git clone https://github.com/Rumarino-Team/hydrus-software-stack.git
cd hydrus-software-stack
```

### Build and Run with Hocker
We use **Hocker** (Hydrus Docker), our intelligent deployment tool that automatically detects your platform and configures the optimal Docker environment.

```bash
# Make hocker executable
chmod +x docker/hydrus-docker/hocker

# Run with automatic platform detection
./docker/hydrus-docker/hocker

# Test the installation
./docker/hydrus-docker/hocker --test

# Quick development mode
./docker/hydrus-docker/hocker --dev

# Force CPU-only mode
./docker/hydrus-docker/hocker --force-cpu
```

Hocker automatically detects your platform (CPU, NVIDIA GPU, WSL, or Jetson) and runs the appropriate Docker configuration with intelligent defaults. For detailed options and configurations, see the [Docker README](docker/README.md).

### Launch the Autonomy System
```bash
# Launch the complete autonomy stack
roslaunch autonomy simulation.launch

# Or launch individual components
roslaunch autonomy mission_planner.launch
roslaunch autonomy cv_publishers.launch
roslaunch autonomy controller.launch
```

## 🎯 Key Features

- **Intelligent Deployment**: Hocker automatically detects and configures your platform
- **Mission Planning**: Hierarchical mission execution for competition tasks (Gate, Slalom, Tagging)
- **Computer Vision**: YOLO-based object detection with configurable color filtering
- **Motion Control**: Precise 3-phase movement control (depth → rotation → linear)
- **Hardware Integration**: Arduino serial communication for thruster control
- **Simulation Ready**: Multi-platform simulation support with configuration groups
- **Web Interface**: Real-time monitoring and visualization tools

## 📁 Project Structure

```
hydrus-software-stack/
├── autonomy/           # Main autonomy system (see autonomy/README.md)
├── docker/            # Docker configurations and deployment
│   └── hydrus-docker/ # Hocker deployment tool
├── embedded_arduino/   # Arduino firmware for hardware control
├── DVL/               # Doppler Velocity Logger integration
├── Embedded_IMU/      # IMU sensor drivers
├── yolo_models/       # YOLO model files for object detection
└── docs/              # Additional documentation
```

## 🧪 Testing

Run the test suite to verify your installation:

```bash
# Quick test with Hocker
./docker/hydrus-docker/hocker --test

# Manual testing
./run_tests.sh
```

## 🐳 Docker Deployment Options

Hocker provides several configuration groups for different use cases:

```bash
# Development with live code editing
./docker/hydrus-docker/hocker --dev

# Testing and CI/CD
./docker/hydrus-docker/hocker --test

# Competition deployment
./docker/hydrus-docker/hocker --competition

# Simulation with rosbag playback
./docker/hydrus-docker/hocker --simulation
```

## 🤝 Contributing

1. **Read the [Philosophy](PHILOSOPHY.md)** - Understand our development principles
2. **Check the [Issues](../../issues)** - Find something to work on
3. **Follow our standards** - Quality code with tests and documentation
4. **Submit a PR** - Clear description and evidence of testing

We welcome contributions that improve usability, add features, or enhance maintainability!

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 Support

- **General Questions**: Check the [autonomy README](autonomy/README.md)
- **Docker Issues**: See the [docker README](docker/README.md)  
- **Bug Reports**: Open an issue with detailed reproduction steps
- **Feature Requests**: Start a discussion in the repository

---

**Mission**: To become the definitive toolkit for underwater robotics - making autonomous underwater vehicles accessible, maintainable, and powerful for teams worldwide.

