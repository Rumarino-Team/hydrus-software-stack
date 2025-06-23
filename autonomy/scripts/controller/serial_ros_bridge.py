#!/usr/bin/env python3
"""
ROS to Serial Bridge for Hydrus Submarine

This node subscribes to ROS topics used for controlling the submarine's thrusters
and translates them into serial commands for the Arduino.

Subscribed topics:
- /hydrus/thrusters/1, 2, 3, 4: Individual thruster controls
- /hydrus/depth: Depth control motors
- /hydrus/torpedo: Torpedo control

Serial command format:
- T1:value - Thruster 1 with specified PWM value (1000-2000)
- T2:value - Thruster 2 with specified PWM value
- T3:value - Thruster 3 with specified PWM value
- T4:value - Thruster 4 with specified PWM value
- D:value  - Depth motors with specified PWM value
- P:value  - Torpedo with specified PWM value
- C:value  - Camera motor angle (-60 to 60 degrees)
"""

import os
import threading
import time

import rospy
import serial
from std_msgs.msg import Int16
from termcolor import colored


class SerialROSBridge:
    """Bridge between ROS topics and serial commands for Arduino"""

    def __init__(self):
        rospy.init_node("serial_ros_bridge", anonymous=True)

        # Get serial port parameters from ROS params
        self.control_port = rospy.get_param("~control_port", "/dev/hydrus_control")
        baud_rate = rospy.get_param("~baud_rate", 115200)

        # Serial connection
        self.ser = None
        self.is_connected = False
        self._connect_to_control_port(baud_rate)

        # Start serial reading thread
        self.running = True

        # Command logging setup
        self.last_commands = {}  # Track last command for each component
        self.command_counts = {}  # Track command counts
        self.last_log_time = time.time()
        self.log_interval = 5.0  # Log every 5 seconds

        # Start logging thread
        self.log_thread = threading.Thread(target=self._log_commands_periodically)
        self.log_thread.daemon = True
        self.log_thread.start()

        rospy.loginfo("📊 Serial Bridge logging enabled (5-second intervals)")

        # ROS Subscribers - Changed to Int16 for PWM values (1000-2000)
        rospy.Subscriber(
            "/hydrus/thrusters/1", Int16, lambda msg: self.thruster_callback(1, msg)
        )
        rospy.Subscriber(
            "/hydrus/thrusters/2", Int16, lambda msg: self.thruster_callback(2, msg)
        )
        rospy.Subscriber(
            "/hydrus/thrusters/3", Int16, lambda msg: self.thruster_callback(3, msg)
        )
        rospy.Subscriber(
            "/hydrus/thrusters/4", Int16, lambda msg: self.thruster_callback(4, msg)
        )
        rospy.Subscriber("/hydrus/depth", Int16, self.depth_callback)
        rospy.Subscriber("/hydrus/torpedo", Int16, self.torpedo_callback)

        # Register shutdown function
        rospy.on_shutdown(self.shutdown)

    def _connect_to_control_port(self, baud_rate):
        """Connect to SOCAT virtual control port"""
        max_retries = 10
        retry_count = 0

        while retry_count < max_retries and not rospy.is_shutdown():
            try:
                if not os.path.exists(self.control_port):
                    rospy.logwarn(
                        f"Control port {self.control_port} not found, waiting..."
                    )
                    time.sleep(2)
                    retry_count += 1
                    continue

                self.ser = serial.Serial(
                    port=self.control_port, baudrate=baud_rate, timeout=1
                )
                self.is_connected = True
                rospy.loginfo(f"Connected to Arduino via {self.control_port}")
                time.sleep(2)  # Allow Arduino reset
                return True

            except serial.SerialException as e:
                rospy.logwarn(f"Failed to connect to {self.control_port}: {e}")
                time.sleep(2)
                retry_count += 1

        rospy.logerr(f"Failed to connect to control port after {max_retries} attempts")
        return False

    def send_command(self, cmd):
        """Send a command to the Arduino and track it for logging"""
        if not self.is_connected or self.ser is None:
            return False

        try:
            # Add newline terminator to the command and ensure proper formatting
            full_cmd = f"{cmd}\n"
            self.ser.write(full_cmd.encode("utf-8"))
            self.ser.flush()

            # Track command for logging
            self._track_command(cmd)

            # Add a small delay to prevent commands from merging
            time.sleep(0.01)
            return True
        except serial.SerialException:
            self.is_connected = False
            return False

    def _track_command(self, cmd):
        """Track command for periodic logging"""
        try:
            # Parse command to extract component and value
            if ":" in cmd:
                component_part, value = cmd.split(":", 1)

                # Map component codes to readable names
                component_map = {
                    "T1": "Thruster 1",
                    "T2": "Thruster 2",
                    "T3": "Thruster 3",
                    "T4": "Thruster 4",
                    "D": "Depth Motors",
                    "P": "Torpedo",
                    "C": "Camera",
                }

                component_name = component_map.get(component_part, component_part)

                # Update tracking
                self.last_commands[component_name] = value
                self.command_counts[component_name] = (
                    self.command_counts.get(component_name, 0) + 1
                )
        except Exception:
            # If parsing fails, just track the raw command
            self.last_commands["Raw"] = cmd
            self.command_counts["Raw"] = self.command_counts.get("Raw", 0) + 1

    # Topic callbacks - Now accepting PWM values directly
    def thruster_callback(self, thruster_num, msg):
        """Handle thruster control messages with PWM values"""
        cmd = f"T{thruster_num}:{msg.data}"
        self.last_commands[f"T{thruster_num}"] = msg.data  # Update last command
        self.command_counts[f"T{thruster_num}"] = (
            self.command_counts.get(f"T{thruster_num}", 0) + 1
        )
        self.send_command(cmd)

    def depth_callback(self, msg):
        """Handle depth control messages with PWM values"""
        cmd = f"D:{msg.data}"
        self.last_commands["D"] = msg.data  # Update last command
        self.command_counts["D"] = self.command_counts.get("D", 0) + 1
        self.send_command(cmd)

    def torpedo_callback(self, msg):
        """Handle torpedo control messages with PWM values"""
        cmd = f"P:{msg.data}"
        self.last_commands["P"] = msg.data  # Update last command
        self.command_counts["P"] = self.command_counts.get("P", 0) + 1
        self.send_command(cmd)

    def _log_commands_periodically(self):
        """Log command statistics every 5 seconds"""
        while self.running:
            try:
                current_time = time.time()
                if current_time - self.last_log_time >= self.log_interval:
                    self._log_command_summary()
                    self.last_log_time = current_time
                time.sleep(1)  # Check every second
            except Exception:
                # Continue running even if logging fails
                time.sleep(1)

    def _log_command_summary(self):
        """Log a summary of recent commands"""
        if not self.last_commands and not self.command_counts:
            rospy.loginfo("📊 Serial Bridge: No commands sent in last 5 seconds")
            return

        rospy.loginfo("📊 SERIAL BRIDGE STATUS (Last 5 seconds)")
        rospy.loginfo("=" * 50)

        # Log last known values for each component
        if self.last_commands:
            rospy.loginfo("🎯 Current Commands:")
            for component, value in self.last_commands.items():
                count = self.command_counts.get(component, 0)
                rospy.loginfo(f"   {component}: {value} (sent {count} times)")

        # Calculate total commands
        total_commands = sum(self.command_counts.values())
        rospy.loginfo(f"📈 Total commands sent: {total_commands}")
        rospy.loginfo("=" * 50)

        # Reset counters for next interval
        self.command_counts = {}

    def shutdown(self):
        """Clean shutdown procedure"""
        rospy.loginfo("🔌 Shutting down Serial ROS Bridge...")
        self.running = False

        # Stop thrusters with neutral PWM value
        if self.is_connected:
            rospy.loginfo("🛑 Stopping all thrusters...")
            self.send_command("T1:1500")
            self.send_command("T2:1500")
            self.send_command("T3:1500")
            self.send_command("T4:1500")
            self.send_command("D:1500")
            self.send_command("P:1500")

        # Close serial connection
        if self.ser is not None:
            self.ser.close()
            self.is_connected = False
            rospy.loginfo("✅ Serial connection closed")

        # Wait for logging thread to finish
        if hasattr(self, "log_thread") and self.log_thread.is_alive():
            self.log_thread.join(timeout=2)


def main():
    try:
        SerialROSBridge()
        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        pass


if __name__ == "__main__":
    main()
