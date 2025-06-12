# Hydrus Software Stack - Philosophy & Contributions

Welcome to the Hydrus Software Stack! This document outlines our core philosophy, development principles, and contribution guidelines. **Every contributor must read and understand these principles before contributing to the project.**

## Our Vision

The Hydrus project aims to become **the definitive toolkit for underwater robotics and RobSub competitions** - a maintainable, production-ready system that dramatically reduces the friction for teams to build autonomous underwater vehicles.

### Core Goals

🎯 **Zero-Friction Usability**: Make it so easy to use that anyone can get started with minimal setup  
🛠️ **Maintainable for Years**: Build software that remains stable and extensible over time  
🚀 **Fast Development**: Move quickly while maintaining quality through good engineering practices  
🔧 **Comprehensive Toolkit**: Provide solutions for the entire autonomy stack, not just pieces  
📈 **Scalable Architecture**: Design systems that grow with user needs and team sizes  

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

**Performance Rule**: Before implementing in another language, you must:
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

### 5. Multi-Platform Integration Philosophy

**Why limit ourselves to one simulation environment?**

Support multiple platforms and tools:
- **Simulation**: Gazebo, Nvidia Omniverse, Unity, Godot, custom simulators
- **Hardware**: Various camera systems, IMUs, thrusters, controllers
- **Deployment**: Docker, bare metal, embedded systems, cloud

**Design Principle**: "It just works" regardless of the user's preferred environment

## Contribution Standards

### Documentation Requirements

Every contribution must include appropriate documentation:

#### 📁 **Folders/Packages**
- Each folder must have a `README.md` explaining its purpose and contents
- Include usage examples and integration points

#### 🚀 **Scripts/Binaries**
- Clear docstrings explaining purpose, inputs, outputs
- Command-line help and usage examples
- Integration instructions with the broader system

#### 🔧 **Functions/Classes**
- Comprehensive docstrings with type hints
- Example usage in docstrings
- Clear parameter descriptions

### Code Quality Standards

#### ✅ **Testing Requirements**
```python
# Every new feature requires:
- Unit tests for individual components
- Integration tests for system interactions
- End-to-end tests for complete workflows
- Performance benchmarks when relevant
```

#### 📝 **Pull Request Standards**
- **Clear Description**: What does this PR do and why?
- **Testing Evidence**: Screenshots, test results, or demonstration videos
- **Breaking Changes**: Clearly document any breaking changes
- **Documentation**: Include or update relevant documentation
- **Small & Focused**: Keep PRs focused on a single feature/fix

#### 🐛 **Issue Standards**
- **Clear Problem Statement**: What's broken or missing?
- **Reproduction Steps**: How to reproduce the issue
- **Expected vs Actual Behavior**: What should happen vs what happens
- **Environment Details**: OS, hardware, software versions
- **Proposed Solution**: Ideas for fixing (when applicable)

### Development Workflow

#### 🏃 **Fast Iteration Process**
1. **Prototype Quickly**: Get working version fast
2. **Add Tests**: Ensure it doesn't break
3. **Document**: Make it usable by others
4. **Refactor**: Improve code quality continuously
5. **Integrate**: Connect with existing systems

#### 🔄 **Continuous Improvement**
- **Refactoring Welcome**: Always improve readability and maintainability
- **Performance Optimization**: After correctness and clarity
- **Architecture Evolution**: Big improvements through small, tested changes

## Technical Standards

### Logging Philosophy
```python
# Every component should provide rich logging
import logging
logger = logging.getLogger(__name__)

# Log state changes, decisions, and errors
logger.info("Starting mission: %s", mission_name)
logger.debug("Processing waypoint: %s", waypoint)
logger.error("Failed to connect to hardware: %s", error)
```

### Error Handling
```python
# Graceful degradation and clear error messages
try:
    result = risky_operation()
except SpecificException as e:
    logger.error("Operation failed: %s", e)
    # Provide fallback or clear guidance
    return fallback_result()
```

### Configuration Management
```python
# Use configuration files, not hardcoded values
config = load_config("mission_config.yaml")
threshold = config.get("detection_threshold", 0.5)
```

## Visualization & Debugging Tools

**Every major component should provide visualization capabilities:**

- **Real-time Monitoring**: Live dashboards for system state
- **Data Recording**: Capture data for offline analysis
- **Debug Interfaces**: Tools to inspect internal state
- **Performance Metrics**: Timing, resource usage, success rates

Example tools we encourage:
- Web-based status dashboards
- RViz visualizations for spatial data
- Command-line monitoring tools
- Jupyter notebooks for data analysis

## Scaling Strategy

### Architecture Principles
- **Modular Design**: Independent components with clear interfaces
- **Service-Oriented**: Use services/APIs instead of tight coupling
- **Configuration-Driven**: Behavior controlled by config files
- **Plugin Architecture**: Easy to add new sensors, algorithms, missions

### Team Scaling
- **Clear Ownership**: Each module has maintainers
- **Onboarding Documentation**: New contributors can get started quickly
- **Review Process**: Code review ensures quality and knowledge sharing
- **Mentorship**: Experienced contributors help newcomers

## What We Welcome

### 🎉 **Encouraged Contributions**
- **New Features**: Especially those that solve real competition problems
- **Integration Tools**: Connect with new hardware, software, or platforms
- **Testing Infrastructure**: Better testing tools and frameworks
- **Documentation**: Tutorials, examples, API documentation
- **Performance Improvements**: After proper benchmarking
- **Developer Tools**: Make development easier and faster
- **Bug Fixes**: Always welcome with proper testing

### ⚡ **High-Priority Areas**
- **End-to-End Testing**: Complete mission simulation and validation
- **Hardware Abstraction**: Make it easy to swap hardware components
- **Simulation Integration**: Support for multiple simulation environments
- **Mission Planning Tools**: Visual mission editors and debugging tools
- **Data Collection**: Tools for gathering and analyzing performance data

## Getting Started

### For New Contributors
1. **Read this document thoroughly**
2. **Set up the development environment** (see main README)
3. **Run the test suite** to ensure everything works
4. **Pick a small issue** labeled "good first issue"
5. **Follow the contribution workflow**
6. **Ask questions** - we're here to help!

### For Reviewers
- **Be constructive**: Focus on making the code better
- **Be educational**: Help contributors learn our standards
- **Be responsive**: Quick feedback keeps momentum
- **Be thorough**: Catch issues before they reach main branch

## Long-Term Vision

We're building more than just competition software - we're creating:

🌊 **The Standard**: The go-to framework for underwater robotics development  
🧰 **The Toolkit**: Comprehensive solutions for autonomy challenges  
🎓 **The Platform**: A foundation for research and education  
📚 **The Knowledge Base**: Accumulated wisdom from years of competition experience  

**Success Metric**: When new teams can go from zero to competitive in weeks, not months, because they're building on our foundation.

---


*"The best robotics software is invisible - it just works, scales effortlessly, and makes impossible things seem easy."*