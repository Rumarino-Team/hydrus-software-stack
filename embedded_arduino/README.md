# Embedded Arduino - Hydrus Hardware Control

This directory contains the Arduino firmware for the Hydrus underwater vehicle hardware control system. The firmware handles thruster control, depth management, torpedo systems, and camera positioning through serial communication with the main ROS system.

## Overview

The Hydrus Arduino system provides low-level hardware control for:
- **8 T100 Thrusters**: Propulsion and maneuvering
- **Depth Control**: Vertical movement motors
- **Torpedo System**: Projectile launch mechanisms
- **Camera Motor**: Pan/tilt camera positioning
- **Serial Communication**: Command interface with ROS

## Hardware Configuration

### Thruster Layout
```
Thruster Layout (9 motors total):
┌─────────────────────────────────┐
│  T1(Pin1)  ←→  T5(Pin5)        │
│     ↕            ↕             │
│  T2(Pin2)  ←→  T6(Pin6)        │
│     ↕            ↕             │
│  T3(Pin3)  ←→  T7(Pin7)        │
│     ↕            ↕             │
│  T4(Pin4)  ←→  T8(Pin8)        │
│                                 │
│       T9(Pin9) - Camera        │
└─────────────────────────────────┘
```

### Motor Assignments
| Motor | Pin | Type | Direction | Function |
|-------|-----|------|-----------|----------|
| T1 | 1 | T100 | Forward | Movement/Rotation |
| T2 | 2 | T100 | Forward | Movement/Rotation |
| T3 | 3 | T100 | Forward | Movement/Rotation |
| T4 | 4 | T100 | Backward | Movement/Rotation |
| T5 | 5 | T100 | Backward | Depth Control |
| T6 | 6 | T100 | Backward | Depth Control |
| T7 | 7 | T100 | Backward | Torpedo System |
| T8 | 8 | T100 | Forward | Torpedo System |
| T9 | 9 | Servo | Forward | Camera Pan/Tilt |

## Installation & Setup

### Arduino IDE Requirements
Install the following libraries in Arduino IDE:
- **Servo Library** (built-in)

### Installation Steps
1. **Open Arduino IDE**
2. **Load Firmware**:
   - Open `Hydrus/Hydrus.ino`
   - Select your Arduino board and port
   - Upload the sketch

### Hardware Connections
- **Power**: Ensure adequate power supply for all 8 T100 thrusters
- **Serial**: Connect Arduino to main computer via USB (typically `/dev/ttyACM0`)
- **ESCs**: Connect each thruster to its designated pin via ESC
- **Camera Servo**: Connect to pin 9

## Serial Communication Protocol

### Command Format
All commands follow the format: `<TYPE><ID>:<VALUE>\n`

### Thruster Commands
```bash
# Individual thrusters (PWM 1000-2000, 1500=neutral)
T1:1600    # Thruster 1 forward
T2:1400    # Thruster 2 reverse
T3:1500    # Thruster 3 neutral
T4:1700    # Thruster 4 forward
```

### Depth Control
```bash
# Controls both depth motors (T5, T6) simultaneously
D:1600     # Ascend
D:1400     # Descend
D:1500     # Neutral buoyancy
```

### Torpedo System
```bash
# Controls both torpedo motors (T7, T8) simultaneously
P:1600     # Fire torpedoes
P:1500     # Neutral
P:1400     # Retract (if applicable)
```

### Camera Control
```bash
# Camera servo angle (-60 to 60 degrees)
C:30       # Pan right 30 degrees
C:-45      # Pan left 45 degrees
C:0        # Center position
```

### PWM Value Ranges
- **Valid Range**: 1000-2000 microseconds
- **Neutral**: 1500 (zero thrust)
- **Forward/Up**: 1501-2000 (increasing thrust)
- **Reverse/Down**: 1000-1499 (increasing reverse thrust)

## Integration with ROS

### ROS Bridge Communication
The Arduino communicates with ROS through the `serial_ros_bridge.py` node:

```bash
# Start the ROS bridge
rosrun autonomy serial_ros_bridge.py
```

### ROS Topics
The bridge subscribes to these topics:
- `/hydrus/thrusters/1` to `/hydrus/thrusters/4`: Individual thruster control
- `/hydrus/depth`: Depth motor control
- `/hydrus/torpedo`: Torpedo system control

### Message Types
All thruster topics use `std_msgs/Int16` with PWM values (1000-2000)

## Safety Features

### PWM Limiting
```cpp
if (pwmValue < 1000) pwmValue = 1000;  // Enforce minimum
if (pwmValue > 2000) pwmValue = 2000;  // Enforce maximum
```

### Initialization Safety
- All thrusters initialize to neutral (1500) on startup
- Commands ignored until initialization complete
- Status verification through serial monitoring

### Direction Inversion
Backward-configured thrusters automatically invert PWM values:
```cpp
if (thrusterArr[id-1].forward) {
    pwm = pwmValue;
} else {
    int diff = PWM_NEUTRAL - pwmValue;
    pwm = PWM_NEUTRAL + diff;
}
```

## Debugging & Monitoring

### Serial Output
The Arduino provides comprehensive status information:

```
=== THRUSTER PIN VALUES ===
Thruster | Pin | PWM Value | Direction
------------------------------------
1 | 1 | 1500 | Forward
2 | 2 | 1500 | Forward
3 | 3 | 1500 | Forward
...
============================
```

### Debug Information
- **Initialization Status**: Startup sequence confirmation
- **Command Processing**: Real-time command acknowledgment
- **PWM Values**: Current thruster states every 5 seconds
- **Error Messages**: Invalid commands and safety violations

### Manual Testing
```bash
# Connect to Arduino directly
screen /dev/ttyACM0 115200

# Send test commands
T1:1600
D:1400
C:30
```

## Troubleshooting

### Common Issues

#### No Response to Commands
```bash
# Check serial connection
ls /dev/ttyACM*

# Verify Arduino is running
screen /dev/ttyACM0 115200
# Should see status messages every 5 seconds
```

#### Thrusters Not Moving
- **Check Power**: Verify adequate power supply for ESCs
- **Check Connections**: Ensure all ESC signal wires connected properly
- **Check PWM Range**: Verify commands use 1000-2000 range
- **Check Initialization**: Wait for "All thrusters initialized" message

#### Serial Communication Issues
- **Baud Rate**: Ensure 115200 baud rate
- **Permissions**: Add user to dialout group on Linux
- **USB Cable**: Try different USB cable/port
- **Arduino IDE**: Re-upload firmware if necessary

#### Wrong Thruster Response
- **Pin Mapping**: Verify thruster connected to correct pin
- **Direction**: Check if thruster should be forward/backward configured
- **PWM Values**: Test with known good PWM values (1400, 1600)

### Testing Commands
```bash
# Test each thruster individually
T1:1600   # Should move thruster 1 forward
T1:1500   # Should stop thruster 1
T1:1400   # Should move thruster 1 backward

# Test depth control
D:1600    # Should activate both depth motors
D:1500    # Should stop depth motors

# Test camera
C:45      # Should pan camera right
C:0       # Should center camera
C:-45     # Should pan camera left
```

## Development Notes

### Performance Considerations
- Commands processed in main loop for real-time response
- PWM values applied continuously, not just on command changes
- Status reporting limited to 5-second intervals to prevent serial flooding

### Future Enhancements
- **Sensor Integration**: IMU and pressure sensor feedback
- **Autonomous Safety**: Automatic depth limiting and collision avoidance
- **Performance Monitoring**: Thruster current and temperature monitoring
- **Wireless Communication**: ESP32 integration for wireless control

---

## Support

For Arduino-specific issues:
- Check serial monitor output for error messages
- Verify all hardware connections
- Test with manual commands before ROS integration
- Review PWM value ranges and safety limits

For ROS integration issues, see the main [autonomy documentation](../autonomy/README.md).
