#!/usr/bin/env python3

import math
import rospy
import actionlib
import time
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Twist
from std_msgs.msg import Int16, Float32, Int16MultiArray
from dataclasses import dataclass, field
from typing import List
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointFeedback, NavigateToWaypointResult
from autonomy.srv import FireTorpedo, FireTorpedoResponse


# ASCII representation of the position of the thrusters:
#  
#  1 *            * 5
#     \          /
#      |________|        
#  2*--|        |--* 6
#      |        |
#      |        |
#  3*--|________|--* 7 
#      |        |
#  4 */          \* 8
#    

class ProportionalController:

    @dataclass(frozen=True)
    class Constants:
        # PWM settings for thruster control
        PWM_NEUTRAL: int = 1500       # Neutral position
        PWM_MIN: int = 1300           # Max reverse
        PWM_MAX: int = 1700           # Max forward
        PWM_TORPEDO_FIRE: int = 1800  # Value to fire torpedo

        DEPTH_PWM_ADJUST: int = 50       # PWM adjustment for depth
        ROTATION_PWM_ADJUST: int = 50    # PWM adjustment for rotation
        LINEAR_PWM_ADJUST: int = 50      # PWM adjustment for linear movement
        
        DISTANCE_THRESHOLD: float = 0.05             # Distance threshold (increased from 0.01 to prevent controller from never stopping)
        
        TOTAL_THRUSTERS: int = 8
        DEPTH_MOTORS_ID: list = field(default_factory=lambda: [2, 7])
        FRONT_MOTORS_ID: list = field(default_factory=lambda: [1, 5])
        BACK_MOTORS_ID: list = field(default_factory=lambda: [4, 8])
        TORPEDO_MOTORS_ID: list = field(default_factory=lambda: [3, 6])
        
        TORPEDO_FIRE_DURATION: float = 1.0  # Time in seconds to keep the torpedo motors at fire PWM

    def __init__(self):
        # Create an instance of Constants for access to class attributes
        self.const = self.Constants()
        
        # ////////////////////////////////////
        # ////////ROS MUTABLE OBJECTS/////////
        # ////////////////////////////////////
        
        self.thruster_pubs = {}
        self.server = None
        self.target_point = None
        self.submarine_pose = None
        self.thruster_values = [self.const.PWM_NEUTRAL for _ in range(8)]
        rospy.loginfo(f"Running with PWM values: {self.thruster_values}")
        self.moving: List[bool] = [False, False, False]  # [depth, rotation, linear]

        #//////////////////////////////////// 
        #////////// INIT ROS DATA////////////
        #////////////////////////////////////
        
        # Publishers changed to Int16 for PWM values
        for i in range(self.const.TOTAL_THRUSTERS):
            self.thruster_pubs[i] = rospy.Publisher(f"/hydrus/thrusters/{i+1}", Int16, queue_size=10)
            
        # Add depth and torpedo publishers
        self.depth_pub = rospy.Publisher("/hydrus/depth", Int16, queue_size=10)
        self.torpedo_pub = rospy.Publisher("/hydrus/torpedo", Int16, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("/submarine/cmd_vel", Twist, queue_size=10)
        
        # Add publishers for controller state monitoring
        self.target_pub = rospy.Publisher("/controller/target_point", Point, queue_size=10)
        self.moving_state_pub = rospy.Publisher("/controller/moving_state", Int16MultiArray, queue_size=10)
        self.target_distance_pub = rospy.Publisher("/controller/target_distance", Float32, queue_size=10)
        
        def imu_pose_callback(msg):
            self.submarine_pose = msg
            
        rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, imu_pose_callback)
        self.server = actionlib.SimpleActionServer('controller_action', NavigateToWaypointAction, self.execute_callback, False)
        self.server.start()
        
        # Initialize the torpedo firing service
        rospy.Service('fire_torpedo', FireTorpedo, self.handle_fire_torpedo)
        rospy.loginfo("Torpedo firing service initialized and ready")
        
    def handle_fire_torpedo(self, req):
        """Handle torpedo firing service requests"""
        torpedo_number = req.torpedo_number
        
        if torpedo_number not in [1, 2]:
            return FireTorpedoResponse(
                success=False,
                message=f"Invalid torpedo number: {torpedo_number}. Must be 1 or 2."
            )
        
        try:
            rospy.loginfo(f"Firing torpedo {torpedo_number}")
            
            # Send firing PWM value to torpedo motors
            self.torpedo_pub.publish(Int16(self.const.PWM_TORPEDO_FIRE))
            
            # Wait for the firing duration
            rospy.sleep(self.const.TORPEDO_FIRE_DURATION)
            
            # Reset to neutral PWM
            self.torpedo_pub.publish(Int16(self.const.PWM_NEUTRAL))
            
            rospy.loginfo(f"Torpedo {torpedo_number} fired successfully")
            return FireTorpedoResponse(
                success=True,
                message=f"Torpedo {torpedo_number} fired successfully"
            )
            
        except Exception as e:
            error_msg = f"Failed to fire torpedo {torpedo_number}: {str(e)}"
            rospy.logerr(error_msg)
            return FireTorpedoResponse(
                success=False,
                message=error_msg
            )

    def execute_callback(self, goal): 
        feedback = NavigateToWaypointFeedback()
        result = NavigateToWaypointResult()

        # The target point is set
        self.target_point = goal.target_point
        target_point = goal.target_point
        rospy.loginfo(f"Goal received: target_point=({target_point.x}, {target_point.y}, {target_point.z})")
        
        # Publish the target point
        self.target_pub.publish(self.target_point)
        
        # Initialize movement sequence to start with depth control
        self.moving = [True, False, False]  # [depth, rotation, linear]
        rospy.loginfo("Movement sequence initialized: Depth control first")

        rate = rospy.Rate(10)  # This is set to 10 hz but you can change it if you like

        while not rospy.is_shutdown():
            if self.submarine_pose is None:
                rospy.logwarn("Submarine pose is not available yet.")
                rate.sleep()
                continue

            # Update feedback
            distance = self.calculate_distance(self.submarine_pose.pose.position, target_point)
            feedback.success = distance < self.const.DISTANCE_THRESHOLD  # Update feedback so you know how close it is
            rospy.loginfo(f"Distance to target: {distance:.2f}")

            # Publish feedback
            self.server.publish_feedback(feedback)

            # Check if the target is reached
            if distance - self.const.DISTANCE_THRESHOLD<0.05:
                rospy.loginfo("Target reached.")
                result.distance_to_target = distance
                self.server.set_succeeded(result)  # Goal is reached, YAY!
                #TODO: In real life, it might a be a good idea to make sure submarine stays on point (due to buoyancy)
                self.moving = [False, False, False]
                return

            # Check for preemption
            if self.server.is_preempt_requested():
                rospy.loginfo("Goal preempted by client.")
                result.distance_to_target = distance
                self.server.set_preempted(result)  # Mark the goal as preempted, sad.
                return

            # Move submarine, she needs the treadmill.
            self.move_submarine(self.submarine_pose, target_point)

            rate.sleep()

        # In case of an unexpected exit
        rospy.logwarn("Action server terminated unexpectedly.")
        result.distance_to_target = -1  # Set an error code for distance
        self.server.set_aborted(result)  # Mark the goal as aborted.


    def move_submarine(self,current_pose, target_point):
        if current_pose is None or target_point is None:
            return

        # Use the proper pose attributes consistently
        position = current_pose.pose.position

        if abs(position.z - target_point.z) > self.const.DISTANCE_THRESHOLD:
            self.adjust_depth_motors(current_pose, target_point)
            self.moving[0] = True
        else:
            self.moving[0] = False
        rotation_done = self.adjust_rotation_motors(current_pose, target_point)
        if rotation_done:
            self.adjust_linear_motors(current_pose, target_point)
            self.moving[1] = False
            self.moving[2] = True
        else:
            self.moving[1] = True
            self.moving[2] = False
        
        # Publish thruster values
        self._publish_all()
            
        rospy.loginfo(f"Current position: ({position.x}, {position.y}, {position.z})")

    # ──────────────────────────  PUBLISHERS  ───────────────────────────────
    def _publish_all(self):
        # Individual thrusters
        for i in range(8):
            self.thruster_pubs[i].publish(Int16(self.thruster_values[i]))

        # Depth motors - send average of depth motor values
        depth_pwm = int(sum(self.thruster_values[m-1] for m in self.const.DEPTH_MOTORS_ID) / len(self.const.DEPTH_MOTORS_ID))
        self.depth_pub.publish(Int16(depth_pwm))

        # Torpedo motors - send average of torpedo motor values
        torpedo_pwm = int(sum(self.thruster_values[m-1] for m in self.const.TORPEDO_MOTORS_ID) / len(self.const.TORPEDO_MOTORS_ID))
        self.torpedo_pub.publish(Int16(torpedo_pwm))

        # Publish cmd_vel for visualization/tracking
        self._publish_cmd_vel()
        
        # Publish target point for monitoring
        if self.target_point is not None:
            self.target_pub.publish(Point(self.target_point.x, self.target_point.y, self.target_point.z))
        
        # Publish moving state
        self.moving_state_pub.publish(Int16MultiArray(data=[int(m) for m in self.moving]))
        
        # Publish delta value (distance to target)
        distance = self.calculate_distance(self.submarine_pose.pose.position, self.target_point) if self.submarine_pose and self.target_point else 0.0
        self.target_distance_pub.publish(Float32(distance))
        
    def _publish_cmd_vel(self):
        tw = Twist()

        # Convert PWM values to normalized values for twist message
        # PWM range: PWM_MIN to PWM_MAX, normalize to -1.0 to 1.0
        pwm_range = self.const.PWM_MAX - self.const.PWM_NEUTRAL
        
        # Forward velocity - average of front and back motors
        front_motor_indices = [m-1 for m in self.const.FRONT_MOTORS_ID]
        back_motor_indices = [m-1 for m in self.const.BACK_MOTORS_ID]
        depth_motor_indices = [m-1 for m in self.const.DEPTH_MOTORS_ID]
        
        fwd_pwm = sum(self.thruster_values[m] for m in front_motor_indices + back_motor_indices) / 4.0
        tw.linear.x = (fwd_pwm - self.const.PWM_NEUTRAL) / pwm_range
        
        # Depth velocity - average of depth motors
        depth_pwm = sum(self.thruster_values[m] for m in depth_motor_indices) / 2.0
        tw.linear.z = (depth_pwm - self.const.PWM_NEUTRAL) / pwm_range
        
        # Angular velocity - difference between left and right side motors
        left_pwm = sum(self.thruster_values[m] for m in [front_motor_indices[0], back_motor_indices[0]]) / 2.0
        right_pwm = sum(self.thruster_values[m] for m in [front_motor_indices[1], back_motor_indices[1]]) / 2.0
        tw.angular.z = (right_pwm - left_pwm) / pwm_range
        
        self.cmd_vel_pub.publish(tw)

    def adjust_depth_motors(self, current_pose, target_point):
            # Get the proper position object from the PoseStamped
            position = current_pose.pose.position
            
            # Calculate the depth difference
            dz = target_point.z - position.z
            depth_distance = abs(dz)
            
            rospy.loginfo(f"Depth adjustment: current={position.z:.2f}, target={target_point.z:.2f}, distance={depth_distance:.2f}")
            
            if depth_distance > self.const.DISTANCE_THRESHOLD:
                if dz > 0:  # Need to go up
                    for motor_id in self.const.DEPTH_MOTORS_ID:
                        # Increase PWM for upward motion
                        idx = motor_id - 1  # Convert 1-based to 0-based index
                        self.thruster_values[idx] = self.const.PWM_NEUTRAL + self.const.DEPTH_PWM_ADJUST
                        rospy.loginfo(f"Setting depth motor {motor_id} to upward: {self.const.PWM_NEUTRAL + self.const.DEPTH_PWM_ADJUST}")
                else:  # Need to go down
                    for motor_id in self.const.DEPTH_MOTORS_ID:
                        # Decrease PWM for downward motion
                        idx = motor_id - 1  # Convert 1-based to 0-based index
                        self.thruster_values[idx] = self.const.PWM_NEUTRAL - self.const.DEPTH_PWM_ADJUST
                        rospy.loginfo(f"Setting depth motor {motor_id} to downward: {self.const.PWM_NEUTRAL - self.const.DEPTH_PWM_ADJUST}")
                return False  # Indicate that depth adjustment is not yet complete
            else:
                # Stop depth movement by setting all depth motors to neutral
                for motor_id in self.const.DEPTH_MOTORS_ID:
                    idx = motor_id - 1
                    self.thruster_values[idx] = self.const.PWM_NEUTRAL
                
                rospy.loginfo("Target depth reached, stopping depth motors")
                return True  # Indicate that the target depth has been reached

    def adjust_rotation_motors(self, current_pose, target_point):
            # Get the proper position and orientation objects
            position = current_pose.pose.position
            orientation = current_pose.pose.orientation
            
            target_yaw = self.calculate_yaw_to_target(position, target_point)
            current_yaw = self.calculate_current_yaw(orientation)
            angle_diff = self.normalize_angle(target_yaw - current_yaw)
            
            rospy.loginfo(f"Rotation adjustment: target_yaw={target_yaw:.2f}, current_yaw={current_yaw:.2f}, diff={angle_diff:.2f}")
            
            if abs(angle_diff) > self.const.DISTANCE_THRESHOLD:
                if angle_diff > 0:
                    # Rotate right - differentially adjust PWM values
                    for motor_id in self.const.FRONT_MOTORS_ID:
                        idx = motor_id - 1  # Convert 1-based to 0-based index
                        if idx == self.const.FRONT_MOTORS_ID[0] - 1:
                            self.thruster_values[idx] = self.const.PWM_NEUTRAL - self.const.ROTATION_PWM_ADJUST
                        else:
                            self.thruster_values[idx] = self.const.PWM_NEUTRAL + self.const.ROTATION_PWM_ADJUST
                            
                    for motor_id in self.const.BACK_MOTORS_ID:
                        idx = motor_id - 1  # Convert 1-based to 0-based index
                        if idx == self.const.BACK_MOTORS_ID[0] - 1:
                            self.thruster_values[idx] = self.const.PWM_NEUTRAL - self.const.ROTATION_PWM_ADJUST
                        else:
                            self.thruster_values[idx] = self.const.PWM_NEUTRAL + self.const.ROTATION_PWM_ADJUST
                    
                    rospy.loginfo("Rotating right")
                else:
                    # Rotate left - differentially adjust PWM values
                    for motor_id in self.const.FRONT_MOTORS_ID:
                        idx = motor_id - 1  # Convert 1-based to 0-based index
                        if idx == self.const.FRONT_MOTORS_ID[0] - 1:
                            self.thruster_values[idx] = self.const.PWM_NEUTRAL + self.const.ROTATION_PWM_ADJUST
                        else:
                            self.thruster_values[idx] = self.const.PWM_NEUTRAL - self.const.ROTATION_PWM_ADJUST
                            
                    for motor_id in self.const.BACK_MOTORS_ID:
                        idx = motor_id - 1  # Convert 1-based to 0-based index
                        if idx == self.const.BACK_MOTORS_ID[0] - 1:
                            self.thruster_values[idx] = self.const.PWM_NEUTRAL + self.const.ROTATION_PWM_ADJUST
                        else:
                            self.thruster_values[idx] = self.const.PWM_NEUTRAL - self.const.ROTATION_PWM_ADJUST
                    
                    rospy.loginfo("Rotating left")
                return False
            else:
                rospy.loginfo("Rotation aligned with target")
                return True
            

    def adjust_linear_motors(self, current_pose, target_point):
            # Access the proper position object
            position = current_pose.pose.position
            
            dx = target_point.x - position.x
            dy = target_point.y - position.y
            distance = math.sqrt(dx**2 + dy**2)

            rospy.loginfo(f"Linear movement: distance={distance:.2f}, Distance Threshold={self.const.DISTANCE_THRESHOLD}")

            if distance > self.const.DISTANCE_THRESHOLD:
                # Set front motors to use positive PWM delta (above neutral), the Arduino will handle thruster orientation
                for motor_id in self.const.FRONT_MOTORS_ID:
                    idx = motor_id - 1  # Convert 1-based to 0-based index
                    self.thruster_values[idx] = self.const.PWM_NEUTRAL + self.const.LINEAR_PWM_ADJUST
                    rospy.loginfo(f"Setting front motor {motor_id} to forward: {self.const.PWM_NEUTRAL + self.const.LINEAR_PWM_ADJUST}")
                    
                # Set back motors to use positive PWM delta (above neutral)
                for motor_id in self.const.BACK_MOTORS_ID:
                    idx = motor_id - 1  # Convert 1-based to 0-based index
                    self.thruster_values[idx] = self.const.PWM_NEUTRAL + self.const.LINEAR_PWM_ADJUST
                    rospy.loginfo(f"Setting back motor {motor_id} to forward: {self.const.PWM_NEUTRAL + self.const.LINEAR_PWM_ADJUST}")
                
                rospy.loginfo("Moving forward with differential thrust")
                return False
            else:
                # Stop forward movement by setting all thrusters to neutral
                for motor_id in self.const.FRONT_MOTORS_ID + self.const.BACK_MOTORS_ID:
                    idx = motor_id - 1
                    self.thruster_values[idx] = self.const.PWM_NEUTRAL
                
                rospy.loginfo("Target reached, stopping forward movement")
                return True  # Indicate that the target has been reached

    @staticmethod
    def calculate_yaw_to_target(current_position, target_position):
        """Calculate yaw angle to target position
        
        Args:
            current_position: Current position (pose.position)
            target_position: Target position (can be a Point or pose.position)
        """
        # Handle different target_position types
        target_x = target_position.x if hasattr(target_position, 'x') else target_position.position.x
        target_y = target_position.y if hasattr(target_position, 'y') else target_position.position.y
        
        dx = target_x - current_position.x
        dy = target_y - current_position.y
        return math.atan2(dy, dx)

    @staticmethod
    def calculate_current_yaw(orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return math.atan2(siny_cosp, cosy_cosp)
    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    @staticmethod
    def calculate_distance(pos1, pos2):
        return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)


def main():
    rospy.init_node('submarine_controller')
    ProportionalController()
    rospy.spin()

if __name__ == '__main__':
    main()

