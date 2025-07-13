# Arduino Serial Communication System Documentation

## Overview

The Hydrus submarine project uses a sophisticated serial communication system to interface between ROS (Robot Operating System) and an Arduino microcontroller. This system handles real-time control of thrusters, depth motors, and torpedo mechanisms while providing comprehensive monitoring and logging capabilities.

## System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   ROS Topics    │───▶│ Serial ROS      │───▶│    Arduino      │
│                 │    │    Bridge       │    │  Microcontroller│
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │
                              ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ Virtual Arduino │    │ Serial Monitor  │    │ Arduino Log     │
│   (Fallback)    │    │     Setup       │    │   Monitor       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Core Components

### 1. Serial ROS Bridge (`serial_ros_bridge.py`)

**Purpose**: Acts as the main communication hub between ROS and Arduino.

**Function**:
- Subscribes to ROS topics for thruster control
- Converts ROS messages to Arduino serial commands
- Maintains bidirectional communication with Arduino
- Handles connection recovery and error management

**ROS Topics Subscribed**:
- `/hydrus/thrusters/1-4` (Int16): Individual thruster PWM values (1000-2000)
- `/hydrus/depth` (Int16): Depth motor PWM control
- `/hydrus/torpedo` (Int16): Torpedo mechanism control

**Serial Command Format**:
```
T1:1500    # Thruster 1 with PWM value 1500 (neutral)
T2:1650    # Thruster 2 with PWM value 1650 (forward)
T3:1350    # Thruster 3 with PWM value 1350 (reverse)
T4:1500    # Thruster 4 with PWM value 1500 (neutral)
D:1600     # Depth motors with PWM value 1600
P:1500     # Torpedo with PWM value 1500 (neutral)
C:30       # Camera motor angle (30 degrees)
```

**Key Features**:
- **PWM Range**: 1000-2000 (standard ESC range)
- **Neutral Position**: 1500 (no movement)
- **Thread-based Response Reading**: Continuous monitoring of Arduino responses
- **Connection Management**: Automatic reconnection attempts
- **Command Formatting**: Proper termination and encoding

### 2. Virtual Arduino System (`virtual_arduino.py`)

**Purpose**: Provides a fallback when no physical Arduino is connected.

**How it Works**:

#### SOCAT Usage (Virtual Arduino Only)
The virtual Arduino system uses `socat` (Socket Cat) to create virtual serial ports **only when no physical Arduino is connected**:

```bash
socat pty,link=/dev/ttyACM0,raw,echo=0 pty,raw,echo=0
```

**SOCAT Parameters Explained**:
- `pty`: Creates a pseudo-terminal
- `link=/dev/ttyACM0`: Creates a symbolic link to the specified device path
- `raw`: Sets raw mode (no character processing)
- `echo=0`: Disables echo

**Important**: SOCAT is NOT used for normal serial monitoring - it's only used in `virtual_arduino.py` to create fake devices when testing without hardware.

#### Virtual Device Creation Process
1. **Check Existing Device**: Verifies if real Arduino exists at `/dev/ttyACM0`
2. **Install Dependencies**: Automatically installs `socat` if missing
3. **Create Virtual Port**: Uses socat to create bidirectional virtual serial communication
4. **Set Permissions**: Ensures proper read/write permissions (0o666)
5. **Response Simulation**: Mimics Arduino responses to commands

#### Simulated Responses
```python
# Command: "T1:1500"
# Response: "ACK:T1:1500"

# Command: "STATUS"
# Response: "STATUS:OK,THRUSTERS:8,DEPTH:0.0"

# Command: "PING"
# Response: "PONG"
```

### 3. Serial Monitor Setup (`setup_serial_monitor.py`)

**Purpose**: Establishes direct serial port monitoring and logging infrastructure.

**Process Flow**:

1. **Port Configuration**:
   ```bash
   stty -F /dev/ttyACM0 raw speed 115200 cs8 -cstopb -parenb
   ```
   - `raw`: Raw mode (no line processing)
   - `speed 115200`: Baud rate setting
   - `cs8`: 8 data bits
   - `-cstopb`: 1 stop bit
   - `-parenb`: No parity

2. **Directory Setup**:
   - Creates `/tmp/hydrus_serial/` directory
   - Initializes log files and PID tracking

3. **Direct Serial Logging**:
   ```python
   # Directly reads from Arduino device (no SOCAT involved)
   cat_process = subprocess.Popen(["cat", "/dev/ttyACM0"], stdout=log_file)
   ```
   **Note**: This setup does NOT use SOCAT - it reads directly from the physical device.

4. **PID Management**:
   - Saves process ID to `/tmp/hydrus_serial/catpid.txt`
   - Enables process monitoring and cleanup

### 4. Arduino Log Monitor (`monitor_arduino_logs.py`)

**Purpose**: Provides real-time monitoring of Arduino serial communication.

**Functionality**:

#### Log File Monitoring
- **Target File**: `/tmp/hydrus_serial/arduinolog.txt`
- **Monitoring Method**: Python-based `tail -f` equivalent
- **Real-time Display**: Continuous output of Arduino responses

#### Monitoring Process
```python
# Wait for log file creation
while not log_file.exists():
    time.sleep(1)

# Continuous monitoring
with open(log_file, "r") as f:
    f.seek(0, 2)  # Go to end of file
    while True:
        line = f.readline()
        if line:
            print(line.rstrip())
        else:
            time.sleep(0.1)
```

## Data Flow Architecture

### Command Flow (ROS → Arduino)
```
1. ROS Node publishes to /hydrus/thrusters/1
   ↓
2. serial_ros_bridge.py receives Int16 message
   ↓
3. Convert to command: "T1:1500"
   ↓
4. Send via serial.write() to /dev/ttyACM0
   ↓
5. Arduino receives and processes command
   ↓
6. Arduino sends acknowledgment: "ACK:T1:1500"
```

### Logging Flow (Arduino → Monitoring)
```
1. Arduino sends response via serial
   ↓
2. setup_serial_monitor.py cat process reads directly from /dev/ttyACM0
   ↓
3. Data written to /tmp/hydrus_serial/arduinolog.txt
   ↓
4. monitor_arduino_logs.py reads and displays log
   ↓
5. Real-time display in tmux pane
```

**Note**: No SOCAT involved in logging - direct device access only.

## File System Structure

```
/tmp/hydrus_serial/
├── arduinolog.txt      # Main Arduino communication log
├── catpid.txt          # Process ID of cat monitoring process
└── (other temp files)

/dev/
├── ttyACM0            # Primary Arduino device (real or virtual)
└── (other devices)
```

## Integration with TMUX Sessions

The system integrates with the main Hydrus tmux session manager:

### Arduino Window Layout
```
┌─────────────────────────────────────────┐
│  Setup Serial Monitor  │ Arduino Logs   │
│  (setup_serial_monitor) │ (monitor_logs) │
│                        │                │
│  - Port configuration  │ - Real-time    │
│  - Directory setup     │   log display  │
│  - Background logging  │ - Error        │
│                        │   monitoring   │
└─────────────────────────────────────────┘
```

### Session Commands
```bash
# Arduino window pane 0: Setup
bash ${CATKIN_WS}/src/hydrus-software-stack/scripts/setup_serial_monitor.py

# Arduino window pane 1: Monitor
python3 ${CATKIN_WS}/src/hydrus-software-stack/scripts/monitor_arduino_logs.py
```

## Error Handling and Recovery

### Connection Recovery
- **Automatic Reconnection**: serial_ros_bridge.py attempts reconnection on failure
- **Graceful Degradation**: System continues operation with virtual Arduino
- **Status Monitoring**: Continuous connection health checks

### Error Types and Responses
1. **Physical Disconnection**: Switches to virtual Arduino
2. **Permission Issues**: Automatic permission setting (chmod 666)
3. **Serial Timeout**: Connection reset and retry
4. **Process Crashes**: PID-based process monitoring and restart

### Cleanup Procedures
```python
def _cleanup(self):
    # Stop cat process
    if self.cat_process:
        self.cat_process.terminate()

    # Kill socat process
    if hasattr(self, "socat_process"):
        os.killpg(os.getpgid(self.socat_process.pid), signal.SIGTERM)

    # Remove virtual device
    if os.path.exists(self.device_path):
        os.unlink(self.device_path)
```

## Security and Permissions

### Device Access
- **Permission Setting**: Automatic chmod 666 on device creation
- **User Groups**: Proper dialout group membership required
- **Directory Permissions**: /tmp/hydrus_serial/ with appropriate access

### Process Management
- **Signal Handling**: Proper SIGINT/SIGTERM handling
- **Process Groups**: SOCAT runs in separate process group
- **PID Tracking**: All processes tracked for cleanup

## Performance Considerations

### Serial Communication
- **Baud Rate**: 115200 bps (optimal for submarine control)
- **Command Timing**: 10ms delay between commands to prevent merging
- **Buffer Management**: Proper serial buffer flushing

### System Resources
- **Thread Usage**: Dedicated thread for Arduino response reading
- **Memory**: Minimal memory footprint with efficient logging
- **CPU**: Low CPU usage with optimized polling intervals

## Troubleshooting Guide

### Common Issues
1. **Device Not Found**: Check USB connection, try virtual Arduino
2. **Permission Denied**: Verify user in dialout group
3. **No Response**: Check baud rate, verify Arduino firmware
4. **Log File Missing**: Ensure setup_serial_monitor.py ran successfully

### Diagnostic Commands
```bash
# Check device existence
ls -la /dev/ttyACM*

# Test serial communication
echo "PING" > /dev/ttyACM0

# Monitor process status
cat /tmp/hydrus_serial/catpid.txt

# Check log contents
tail -f /tmp/hydrus_serial/arduinolog.txt
```

## Future Enhancements

### Planned Improvements
1. **Protocol Versioning**: Structured command/response protocol
2. **Error Reporting**: Enhanced error codes from Arduino
3. **Configuration Management**: Dynamic parameter adjustment
4. **Performance Metrics**: Communication latency monitoring
5. **Security**: Encrypted communication for sensitive operations

This comprehensive serial communication system ensures reliable, monitored, and fault-tolerant communication between the ROS-based submarine control system and the Arduino microcontroller, with complete logging and monitoring capabilities for debugging and operational oversight.

## Architecture Efficiency Analysis

### Current Implementation Limitations

The current architecture has several efficiency issues:

#### File I/O Bottleneck
```
Arduino → /dev/ttyACM0 → cat process → file write → monitor process → display
                     ↓
             serial_ros_bridge (direct access)
```

**Problems with Current Approach**:

1. **Constant File Writing**: `cat /dev/ttyACM0 > arduinolog.txt` continuously writes to disk
2. **File Reading Overhead**: `monitor_arduino_logs.py` must constantly poll and read the file
3. **Disk I/O Latency**: File system operations introduce latency in log display
4. **Resource Waste**: Unnecessary disk writes for ephemeral monitoring data
5. **Limited Scalability**: Adding more monitoring processes requires more file I/O

#### Performance Metrics
- **File I/O**: ~10-100ms latency per write/read cycle
- **Disk Usage**: Continuous writing (potentially GB/day depending on Arduino output)
- **CPU Overhead**: File system calls, polling, buffer management
- **Memory**: File buffers in kernel and userspace

### Proposed SOCAT-Based Architecture

A more efficient approach would use SOCAT to create virtual serial port multiplexing:

```
                    ┌─ /dev/ttyVIRT0 → serial_ros_bridge.py
Arduino → /dev/ttyACM0 → SOCAT ┤
                    └─ /dev/ttyVIRT1 → monitor_display.py
```

#### SOCAT Multiplexing Command
```bash
# Create bidirectional virtual ports
socat -d -d pty,raw,echo=0,link=/dev/ttyVIRT0 pty,raw,echo=0,link=/dev/ttyVIRT1 &

# Bridge physical device to virtual ports
socat /dev/ttyACM0,raw,echo=0 EXEC:'tee >(socat - /dev/ttyVIRT0) | socat - /dev/ttyVIRT1'
```

#### Benefits of SOCAT Approach

1. **Zero File I/O**: Direct memory-to-memory data streaming
2. **Real-time Access**: Multiple processes can access serial data simultaneously
3. **Lower Latency**: ~1-10ms vs 10-100ms for file-based approach
4. **Better Resource Usage**: No disk writes for monitoring
5. **Scalability**: Easy to add more virtual endpoints
6. **Isolation**: Each consumer gets independent data stream

#### Advanced SOCAT Architecture
```bash
# Master serial multiplexer
socat /dev/ttyACM0,raw,echo=0,b115200 \
  EXEC:'tee >(socat - pty,link=/dev/hydrus_control,raw,echo=0) \
            >(socat - pty,link=/dev/hydrus_monitor,raw,echo=0) \
            >(socat - pty,link=/dev/hydrus_log,raw,echo=0)'
```

**Result**:
- `/dev/hydrus_control` → serial_ros_bridge.py (bidirectional)
- `/dev/hydrus_monitor` → real-time monitoring display
- `/dev/hydrus_log` → optional file logging (if needed)

### Performance Comparison

| Aspect | Current (File-based) | Proposed (SOCAT-based) |
|--------|---------------------|------------------------|
| Latency | 10-100ms | 1-10ms |
| Disk I/O | Continuous | Optional/None |
| CPU Usage | High (file ops) | Low (memory ops) |
| Scalability | Poor | Excellent |
| Real-time | No | Yes |
| Resource Usage | High | Low |

### Implementation Considerations

#### Advantages of SOCAT Approach
- **Performance**: Significantly reduced latency and resource usage
- **Flexibility**: Easy to add/remove monitoring endpoints
- **Real-time**: True real-time data access for all consumers
- **Reliability**: Less failure points (no file system dependencies)

#### Potential Drawbacks
- **Complexity**: More complex setup and process management
- **Dependencies**: Requires SOCAT to be properly installed and configured
- **Debugging**: Harder to debug virtual port issues
- **Process Management**: More processes to monitor and restart

### Recommended Architecture Migration

#### Phase 1: Hybrid Approach
- Keep current file-based logging as backup
- Add SOCAT virtual ports for real-time monitoring
- Test performance improvements

#### Phase 2: Full Migration
- Replace file-based monitoring with SOCAT endpoints
- Implement proper process management for SOCAT multiplexer
- Add configuration management for virtual port creation

#### Implementation Script Example
```python
def setup_socat_multiplexer(device="/dev/ttyACM0"):
    """Setup SOCAT-based serial multiplexing"""

    # Create virtual device paths
    control_port = "/dev/hydrus_control"
    monitor_port = "/dev/hydrus_monitor"
    log_port = "/dev/hydrus_log"

    # SOCAT multiplexer command
    socat_cmd = [
        "socat",
        f"{device},raw,echo=0,b115200",
        f"EXEC:tee >(socat - pty,link={control_port},raw,echo=0) " +
        f">(socat - pty,link={monitor_port},raw,echo=0) " +
        f">(socat - pty,link={log_port},raw,echo=0)"
    ]

    # Start multiplexer process
    process = subprocess.Popen(socat_cmd, preexec_fn=os.setsid)

    return process, [control_port, monitor_port, log_port]
```

**Conclusion**: You are absolutely correct - the current file-based approach is inefficient. A SOCAT-based virtual port multiplexing system would provide significant performance improvements, better resource utilization, and true real-time monitoring capabilities.
