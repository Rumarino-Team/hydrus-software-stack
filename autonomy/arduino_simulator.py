#!/usr/bin/env python3

import math
import random
import time

import actionlib
import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from std_msgs.msg import Float32, Int16

from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointGoal


class ArduinoSimulator:
    def __init__(self):
        rospy.init_node("controller_tester", anonymous=True)

        self.pose_publisher = rospy.Publisher(
            "/zed2i/zed_node/pose", PoseStamped, queue_size=10
        )
        self.client = actionlib.SimpleActionClient(
            "controller_action", NavigateToWaypointAction
        )

        rospy.loginfo("Waiting for the controller action server to start...")
        self.client.wait_for_server()
        rospy.loginfo("Controller action server started.")

        # Constants to match those in controllers.py
        self.DELTA = 0.20  # Threshold for considering a step complete
        self.PWM_NEUTRAL = 1500

        # Current simulated position state
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "base_link"
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.0
        self.current_pose.pose.orientation = self.euler_to_quaternion(
            0
        )  # Start facing forward

        # Movement state based on thruster commands
        self.target_distance = float("inf")
        self.moving_vertically = 0  # -1: down, 0: no vertical movement, 1: up
        self.moving_forward = 0  # -1: backward, 0: no forward movement, 1: forward
        self.pwm_front = [0, 0]
        self.pwm_back = [0, 0]
        self.rotating = [0, 0, 0, 0]  # -1: left, 0: no rotation, 1: right
        self.expected_neutral_thrusters = None

        # Position update rates (meters per second or radians per second)
        self.vertical_rate = 0.02  # 2 cm per update at 10Hz = 0.2 m/s
        self.forward_rate = 0.03  # 3 cm per update at 10Hz = 0.3 m/s
        self.rotation_rate = 0.02  # 0.02 radians per update at 10Hz = ~0.2 rad/s

        # Subscribe to thruster commands to update movement state
        self.depth_sub = rospy.Subscriber("/hydrus/depth", Int16, self.depth_callback)
        self.target_sub = rospy.Subscriber(
            "/controller/target_distance", Float32, self.target_distance_callback
        )

        # Subscribe to individual thruster commands to detect rotation and forward movement
        self.thruster_subs = []
        for i in range(1, 9):
            self.thruster_subs.append(
                rospy.Subscriber(
                    f"/hydrus/thrusters/{i}",
                    Int16,
                    lambda msg, idx=i: self.thruster_callback(msg, idx),
                )
            )

        # Timer to update position based on current movement state
        self.update_timer = rospy.Timer(
            rospy.Duration(0.1), self.update_position
        )  # 10 Hz updates

    def target_distance_callback(self, msg):
        """Update the target distance value"""
        self.target_distance = msg.data

    def depth_callback(self, msg):
        """Update vertical movement state based on depth commands"""
        pwm = msg.data
        if abs(pwm - self.PWM_NEUTRAL) < 10:
            self.moving_vertically = 0
        elif pwm > self.PWM_NEUTRAL:
            self.moving_vertically = 1  # Moving up
            rospy.loginfo(f"Simulating upward movement (PWM: {pwm})")
        else:
            self.moving_vertically = -1  # Moving down
            rospy.loginfo(f"Simulating downward movement (PWM: {pwm})")

    def thruster_callback(self, msg, thruster_id):
        """Update rotation and forward movement based on individual thruster commands"""
        pwm = msg.data

        # Real-time thruster validation if we're in test mode
        if self.expected_neutral_thrusters:
            if thruster_id in self.expected_neutral_thrusters:
                if abs(pwm - self.PWM_NEUTRAL) > 10:  # Threshold for "active"
                    error_msg = f"ASSERTION FAILED: Thruster {thruster_id} should stay neutral but got PWM {pwm} (expected ~{self.PWM_NEUTRAL})"
                    rospy.logerr(error_msg)
                    raise AssertionError(error_msg)

        if thruster_id in [2, 3, 6, 7]:
            return

        # Front motors are 1 and 5
        if thruster_id in [1, 5]:
            if abs(pwm - self.PWM_NEUTRAL) < 10:
                return

            # Differential thrust for front motors affects rotation
            if thruster_id == 1:
                self.pwm_front[0] = pwm
                if pwm < self.PWM_NEUTRAL:
                    self.rotating[0] = 1  # Rotating right
                elif pwm > self.PWM_NEUTRAL:
                    self.rotating[0] = -1  # Rotating left
                else:
                    self.rotating[0] = 0
            elif thruster_id == 5:
                self.pwm_front[1] = pwm
                if pwm < self.PWM_NEUTRAL:
                    self.rotating[1] = -1  # Rotating left
                elif pwm > self.PWM_NEUTRAL:
                    self.rotating[1] = 1  # Rotating right
                else:
                    self.rotating[1] = 0

        # Back motors are 4 and 8
        elif thruster_id in [4, 8]:
            if abs(pwm - self.PWM_NEUTRAL) < 10:
                return

            # Differential thrust for back motors affects rotation
            if thruster_id == 4:
                self.pwm_back[0] = pwm
                if pwm < self.PWM_NEUTRAL:
                    self.rotating[2] = 1  # Rotating right
                elif pwm > self.PWM_NEUTRAL:
                    self.rotating[2] = -1  # Rotating left
                else:
                    self.rotating[2] = 0
            elif thruster_id == 8:
                self.pwm_back[1] = pwm
                if pwm < self.PWM_NEUTRAL:
                    self.rotating[3] = -1  # Rotating left
                elif pwm > self.PWM_NEUTRAL:
                    self.rotating[3] = 1  # Rotating right
                else:
                    self.rotating[3] = 0

    def update_position(self, event):
        """Update the simulated position based on current movement state"""
        if not hasattr(self, "current_pose"):
            return

        # Update depth position
        if self.moving_vertically != 0:
            self.current_pose.pose.position.z += (
                self.moving_vertically * self.vertical_rate
            )
            rospy.loginfo(f"Updated depth: {self.current_pose.pose.position.z:.3f} m")

        # Get current yaw from quaternion
        current_orientation = self.current_pose.pose.orientation
        siny_cosp = 2 * (
            current_orientation.w * current_orientation.z
            + current_orientation.x * current_orientation.y
        )
        cosy_cosp = 1 - 2 * (current_orientation.y**2 + current_orientation.z**2)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Update rotation
        net_rotation = sum(self.rotating)
        if net_rotation != 0:
            rotation = 0
            if net_rotation > 0:
                rotation = 1
            else:
                rotation = -1
            new_yaw = current_yaw + (rotation * self.rotation_rate)
            self.current_pose.pose.orientation = self.euler_to_quaternion(new_yaw)
            rospy.loginfo(f"Updated rotation: {math.degrees(new_yaw):.1f}°")
        elif 0 not in self.pwm_front and 0 not in self.pwm_back:
            # Check for several scenarios where in real life forward or backward momentum would ocur
            # It might be a bit overengineered but it works ¯\_(ツ)_/¯
            first_cross_equals = (self.pwm_front[0] == self.pwm_back[1]) and (
                self.pwm_front[0] != self.PWM_NEUTRAL
            )
            second_cross_equals = (self.pwm_front[1] == self.pwm_back[0]) and (
                self.pwm_front[1] != self.PWM_NEUTRAL
            )
            front_equals = (self.pwm_front[0] == self.pwm_front[1]) and (
                self.pwm_front[0] != self.PWM_NEUTRAL
            )
            back_equals = (self.pwm_back[0] == self.pwm_back[1]) and (
                self.pwm_back[0] != self.PWM_NEUTRAL
            )

            if first_cross_equals or front_equals:
                if self.pwm_front[0] > self.PWM_NEUTRAL:
                    self.moving_forward = 1
                elif self.pwm_front[0] < self.PWM_NEUTRAL:
                    self.moving_forward = -1
            elif second_cross_equals:
                if self.pwm_front[1] > self.PWM_NEUTRAL:
                    self.moving_forward = 1
                elif self.pwm_front[1] < self.PWM_NEUTRAL:
                    self.moving_forward = -1
            elif back_equals:
                if self.pwm_back[0] > self.PWM_NEUTRAL:
                    self.moving_forward = 1
                elif self.pwm_back[0] < self.PWM_NEUTRAL:
                    self.moving_forward = -1
            else:
                self.moving_forward = 0
        else:
            self.moving_forward = 0
        # Update forward position based on current orientation and movement direction
        if self.moving_forward != 0:
            # Move in the direction we're facing, with the sign determined by moving_forward
            # If positive, move in the direction we're facing; if negative, move opposite
            forward_direction = self.moving_forward
            self.current_pose.pose.position.x += (
                forward_direction * math.cos(current_yaw) * self.forward_rate
            )
            self.current_pose.pose.position.y += (
                forward_direction * math.sin(current_yaw) * self.forward_rate
            )

        # Always publish the pose to provide continuous position updates
        self.current_pose.header.stamp = rospy.Time.now()
        self.pose_publisher.publish(self.current_pose)

    def euler_to_quaternion(self, yaw, pitch=0, roll=0):
        """
        Convert Euler angles to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    # Unused but maybe useful in the future
    def calculate_yaw_to_target(self, current_position, target_point):
        """
        Calculate the yaw angle to face the target
        """
        dx = target_point.x - current_position.x
        dy = target_point.y - current_position.y
        return math.atan2(dy, dx)
