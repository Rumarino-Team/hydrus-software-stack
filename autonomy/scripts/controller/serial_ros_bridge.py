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
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud_rate = rospy.get_param("~baud_rate", 115200)

        # Serial connection
        self.ser = None
        self.is_connected = False
        self.connect_serial(port, baud_rate)

        # Start serial reading thread
        self.running = True
        self.response_thread = threading.Thread(target=self._read_responses)
        self.response_thread.daemon = True
        self.response_thread.start()

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

    def connect_serial(self, port, baud_rate):
        """Establish a serial connection to the Arduino"""
        try:
            self.ser = serial.Serial(port=port, baudrate=baud_rate, timeout=1)
            self.is_connected = True

            # Allow time for Arduino to reset
            time.sleep(2)
            return True
        except serial.SerialException as e:
            # Error handled silently
            return False

    def send_command(self, cmd):
        """Send a command to the Arduino"""
        if not self.is_connected or self.ser is None:
            return False

        try:
            # Add newline terminator to the command and ensure proper formatting
            full_cmd = f"{cmd}\n"
            self.ser.write(full_cmd.encode("utf-8"))
            self.ser.flush()
            # Add a small delay to prevent commands from merging
            time.sleep(0.01)
            return True
        except serial.SerialException as e:
            self.is_connected = False
            return False

    def _read_responses(self):
        """Thread to read and log responses from the Arduino"""
        while self.running and not rospy.is_shutdown():
            try:
                if self.is_connected and self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8").strip()
            except Exception as e:
                if self.running:
                    self.is_connected = False
            time.sleep(0.1)

    # Topic callbacks - Now accepting PWM values directly
    def thruster_callback(self, thruster_num, msg):
        """Handle thruster control messages with PWM values"""
        cmd = f"T{thruster_num}:{msg.data}"
        self.send_command(cmd)

    def depth_callback(self, msg):
        """Handle depth control messages with PWM values"""
        cmd = f"D:{msg.data}"
        self.send_command(cmd)

    def torpedo_callback(self, msg):
        """Handle torpedo control messages with PWM values"""
        cmd = f"P:{msg.data}"
        self.send_command(cmd)

    def shutdown(self):
        """Clean shutdown procedure"""
        self.running = False

        # Stop thrusters with neutral PWM value
        if self.is_connected:
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

        # Wait for thread to end
        if self.response_thread is not None:
            self.response_thread.join(timeout=1)


def main():
    try:
        bridge = SerialROSBridge()

        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        pass


if __name__ == "__main__":
    main()
