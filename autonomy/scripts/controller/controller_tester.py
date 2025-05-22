#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointGoal
from std_msgs.msg import Int16
import actionlib
import random
import math
import time


class ControllerTester:
    def __init__(self):
        rospy.init_node('controller_tester', anonymous=True)

        self.pose_publisher = rospy.Publisher('/zed2i/zed_node/pose', PoseStamped, queue_size=10)
        self.client = actionlib.SimpleActionClient('controller_action', NavigateToWaypointAction)

        rospy.loginfo("Waiting for the controller action server to start...")
        self.client.wait_for_server()
        rospy.loginfo("Controller action server started.")
        
        # Constants to match those in controllers.py
        self.DELTA = 0.05  # Threshold for considering a step complete
        self.PWM_NEUTRAL = 1500
        
        # Current simulated position state
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "base_link"
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.0
        self.current_pose.pose.orientation = self.euler_to_quaternion(0)  # Start facing forward
        
        # Movement state based on thruster commands
        self.moving_vertically = 0    # -1: down, 0: no vertical movement, 1: up
        self.moving_forward = 0       # -1: backward, 0: no forward movement, 1: forward
        self.rotating = 0             # -1: left, 0: no rotation, 1: right
        
        # Position update rates (meters per second or radians per second)
        self.vertical_rate = 0.02     # 2 cm per update at 10Hz = 0.2 m/s
        self.forward_rate = 0.03      # 3 cm per update at 10Hz = 0.3 m/s
        self.rotation_rate = 0.02     # 0.02 radians per update at 10Hz = ~0.2 rad/s
        
        # Subscribe to thruster commands to update movement state
        rospy.Subscriber('/hydrus/depth', Int16, self.depth_callback)
        
        # Subscribe to individual thruster commands to detect rotation and forward movement
        for i in range(1, 9):
            rospy.Subscriber(f'/hydrus/thrusters/{i}', Int16, 
                            lambda msg, idx=i: self.thruster_callback(msg, idx))
        
        # Timer to update position based on current movement state
        self.update_timer = rospy.Timer(rospy.Duration(0.1), self.update_position)  # 10 Hz updates

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
        
        # Front motors are 1 and 5
        if thruster_id in [1, 5]:
            if abs(pwm - self.PWM_NEUTRAL) < 10:
                return
            
            # Differential thrust for front motors affects rotation
            if thruster_id == 1 and pwm < self.PWM_NEUTRAL:
                self.rotating = 1  # Rotating right
            elif thruster_id == 1 and pwm > self.PWM_NEUTRAL:
                self.rotating = -1  # Rotating left
            elif thruster_id == 5 and pwm < self.PWM_NEUTRAL:
                self.rotating = -1  # Rotating left
            elif thruster_id == 5 and pwm > self.PWM_NEUTRAL:
                self.rotating = 1  # Rotating right
                
            # Front motors BELOW neutral (reverse) help move FORWARD
            if pwm < self.PWM_NEUTRAL:
                self.moving_forward = 1  # Move forward
            else:
                self.moving_forward = -1  # Move backward
                
        # Back motors are 4 and 8
        elif thruster_id in [4, 8]:
            if abs(pwm - self.PWM_NEUTRAL) < 10:
                return
                
            # Differential thrust for back motors affects rotation
            if thruster_id == 4 and pwm < self.PWM_NEUTRAL:
                self.rotating = 1  # Rotating right
            elif thruster_id == 4 and pwm > self.PWM_NEUTRAL:
                self.rotating = -1  # Rotating left
            elif thruster_id == 8 and pwm < self.PWM_NEUTRAL:
                self.rotating = -1  # Rotating left
            elif thruster_id == 8 and pwm > self.PWM_NEUTRAL:
                self.rotating = 1  # Rotating right
                
            # Back motors ABOVE neutral (forward) help move FORWARD
            if pwm > self.PWM_NEUTRAL:
                self.moving_forward = 1  # Move forward
            else:
                self.moving_forward = -1  # Move backward

    def update_position(self, event):
        """Update the simulated position based on current movement state"""
        if not hasattr(self, 'current_pose'):
            return
            
        # Update depth position
        if self.moving_vertically != 0:
            self.current_pose.pose.position.z += self.moving_vertically * self.vertical_rate
            rospy.loginfo(f"Updated depth: {self.current_pose.pose.position.z:.3f} m")
            
        # Get current yaw from quaternion
        current_orientation = self.current_pose.pose.orientation
        siny_cosp = 2 * (current_orientation.w * current_orientation.z + current_orientation.x * current_orientation.y)
        cosy_cosp = 1 - 2 * (current_orientation.y**2 + current_orientation.z**2)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Update rotation
        if self.rotating != 0:
            new_yaw = current_yaw + (self.rotating * self.rotation_rate)
            self.current_pose.pose.orientation = self.euler_to_quaternion(new_yaw)
            rospy.loginfo(f"Updated rotation: {math.degrees(new_yaw):.1f}°")
            
        # Update forward position based on current orientation and movement direction
        if self.moving_forward != 0:
            # Move in the direction we're facing, with the sign determined by moving_forward
            # If positive, move in the direction we're facing; if negative, move opposite
            forward_direction = self.moving_forward
            self.current_pose.pose.position.x += forward_direction * math.cos(current_yaw) * self.forward_rate
            self.current_pose.pose.position.y += forward_direction * math.sin(current_yaw) * self.forward_rate
            rospy.loginfo(f"Updated position: ({self.current_pose.pose.position.x:.3f}, {self.current_pose.pose.position.y:.3f})")
            
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
        
    def calculate_yaw_to_target(self, current_position, target_point):
        """
        Calculate the yaw angle to face the target
        """
        dx = target_point.x - current_position.x
        dy = target_point.y - current_position.y
        return math.atan2(dy, dx)

    def simulate_pose(self, target_point, time_to_reach=10.0):
        """
        Simulates the submarine's movement towards a target point by publishing updated positions.
        Movement follows the same sequence as the controllers.py:
        1. Adjust depth
        2. Rotate to face the target
        3. Move forward in a straight line
        
        Args:
            target_point: The target Point to move towards
            time_to_reach: Total time in seconds it should take to reach the target
        """
        # Initialize pose
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "base_link"
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.0
        self.current_pose.pose.orientation = self.euler_to_quaternion(0)  # Start facing forward (0 radians)

        # Movement states: [depth, rotation, linear]
        moving = [True, False, False]
        
        # Calculate initial distances for each phase
        depth_distance = abs(target_point.z - self.current_pose.pose.position.z)
        
        # Allocate time for each movement phase (proportionally to distances)
        total_time = time_to_reach
        time_per_phase = total_time / 3  # Divide total time into 3 phases
        
        # Calculate step sizes for each phase
        depth_step_size = depth_distance / (time_per_phase * 10) if depth_distance > 0 else 0.1  # 10 Hz rate
        rotation_step_size = math.pi / (time_per_phase * 10)  # Assuming max of PI radians to rotate
        
        # Calculate linear distance (after depth and rotation are complete)
        dx = target_point.x - self.current_pose.pose.position.x
        dy = target_point.y - self.current_pose.pose.position.y
        linear_distance = (dx**2 + dy**2) ** 0.5
        linear_step_size = linear_distance / (time_per_phase * 10) if linear_distance > 0 else 0.1  # 10 Hz rate
        
        rospy.loginfo(f"Simulating movement to reach target in {time_to_reach} seconds")
        rospy.loginfo(f"Movement sequence: Depth → Rotation → Linear")
        
        # NOTE: The rest of this method is just the manual simulation
        # The real position updates will now come from the update_position method
        # based on thruster commands, so we'll just monitor progress here
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Check if we've reached the target
            current_pos = self.current_pose.pose.position
            distance = math.sqrt(
                (current_pos.x - target_point.x)**2 + 
                (current_pos.y - target_point.y)**2 + 
                (current_pos.z - target_point.z)**2
            )
            
            if distance < self.DELTA:
                rospy.loginfo(f"Target reached! Final position: ({current_pos.x:.3f}, {current_pos.y:.3f}, {current_pos.z:.3f})")
                break
                
            # Check if we've timed out (3x the expected time)
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > time_to_reach * 3:
                rospy.logwarn(f"Timeout reached after {elapsed:.1f} seconds. Current distance to target: {distance:.3f}")
                break
                
            # Log current position periodically
            if int(elapsed * 10) % 20 == 0:  # Every ~2 seconds
                rospy.loginfo(f"Current position: ({current_pos.x:.3f}, {current_pos.y:.3f}, {current_pos.z:.3f}), distance: {distance:.3f}")
                
            rate.sleep()

    def test_controller(self, target_point, time_to_reach=10.0):
        """
        Tests the controller by sending a target point and simulating movement.
        
        Args:
            target_point: The target Point to move towards
            time_to_reach: Time in seconds it should take to reach the target
        """
        rospy.loginfo(f"Testing controller with target point: ({target_point.x}, {target_point.y}, {target_point.z})")
        rospy.loginfo(f"Time to reach target: {time_to_reach} seconds")

        goal = NavigateToWaypointGoal()
        goal.target_point = target_point  # Assigning the Point object to the target_point field. Target aquiared. 

        self.client.send_goal(goal)

        rospy.loginfo("Simulating submarine pose updates...")
        self.simulate_pose(target_point, time_to_reach)

        rospy.loginfo("Waiting for result from the controller...")
        self.client.wait_for_result()

        result = self.client.get_result()
        if result is None:
            rospy.logerr("No result received from the controller action server.")
            return

        rospy.loginfo(f"Final distance to target: {result.distance_to_target:.2f}")
        if result.distance_to_target < 0.1:  
            rospy.loginfo("Controller successfully moved the submarine to the target.")
        else:
            rospy.logwarn("Controller failed to reach the target within acceptable range.")

    def run(self, time_to_reach=10.0):
        """
        Runs the tester with a predefined target point.
        
        Args:
            time_to_reach: Time in seconds it should take to reach the target
        """
        #Chooses a random target because testing:)
        target_point = Point()
        target_point.x = random.uniform(1.0, 5.0)
        target_point.y = random.uniform(1.0, 5.0)
        target_point.z = random.uniform(1.0, 3.0)

        self.test_controller(target_point, time_to_reach)


if __name__ == '__main__':
    try:
        import sys
        time_to_reach = 10.0  # Default time
        
        # Check if time argument is provided
        if len(sys.argv) > 1:
            try:
                time_to_reach = float(sys.argv[1])
                if time_to_reach <= 0:
                    rospy.logwarn("Time to reach must be positive. Using default of 10.0 seconds.")
                    time_to_reach = 10.0
            except ValueError:
                rospy.logwarn("Invalid time argument. Using default of 10.0 seconds.")
        
        tester = ControllerTester()
        tester.run(time_to_reach)
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller tester interrupted.")
