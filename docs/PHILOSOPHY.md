# Hydrus Software Stack - Philosophy & Contributions

Welcome to the Hydrus Software Stack! This document outlines our core philosophy, development principles, and contribution guidelines. **Every contributor must read and understand these principles before contributing to the project.**

## Our Vision

The Hydrus project aims to become the  toolkit for underwater robotics and the [RoboSub competition](https://robosub.org/).

## Development Philosophy

### 1. Software Engineering Excellence in Robotics

Most amateur robotics projects suffer from poor software engineering practices. We reject this norm and embrace:

- **Comprehensive Testing**: Unit tests, integration tests, end-to-end testing pipelines
- **Continuous Integration**: Automated testing and validation on every change
- **Documentation-First**: Every feature, script, and package must be thoroughly documented
- **Refactoring Culture**: Constant improvement of code readability and structure
- **Logging & Visualization**: If you can't see it, you can't debug it efficiently

### 2. Python-First Development

**All scripts and primary functionality should be written in Python.**

**Why Python?**
- **Accessibility**: More developers can contribute and maintain the codebase
- **Rapid Prototyping**: Faster iteration and experimentation
- **Rich Ecosystem**: Extensive libraries for robotics, ML, and computer vision
- **Readability**: Self-documenting code that's easier to review and maintain

**Performance Rule**: Before implementing in another language, or add a feature for incrementing performace:
1. **Benchmark** the Python implementation
2. **Verify** it's not an architectural design problem
3. **Prove** the performance gain justifies the complexity cost

### 3. Minimal External Dependencies

**Especially avoid heavy dependence on ROS-specific libraries.**

**Principles**:
- Functionality should work outside the ROS ecosystem when possible
- Before adding a library, ask: "How hard would it be to implement the core functionality we need?"
- Prefer lightweight, focused solutions over heavyweight frameworks
- Make the system portable across different robotics platforms

### 4. Fast Development with Stability

**Current Phase**: Rapid growth and feature development
- Any contribution that adds value without breaking the system is welcome
- New ideas and experimentation are encouraged
- Move fast, but maintain quality through testing

**Future Goal**: Production-ready version with slower, more deliberate changes
