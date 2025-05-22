#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointGoal
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
        self.DELTA = 0.2  # Threshold for considering a step complete

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
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.euler_to_quaternion(0)  # Start facing forward (0 radians)

        # Movement states: [depth, rotation, linear]
        moving = [True, False, False]
        
        # Calculate initial distances for each phase
        depth_distance = abs(target_point.z - pose.pose.position.z)
        
        # Allocate time for each movement phase (proportionally to distances)
        total_time = time_to_reach
        time_per_phase = total_time / 3  # Divide total time into 3 phases
        
        # Calculate step sizes for each phase
        depth_step_size = depth_distance / (time_per_phase * 10) if depth_distance > 0 else 0.1  # 10 Hz rate
        rotation_step_size = math.pi / (time_per_phase * 10)  # Assuming max of PI radians to rotate
        
        # Calculate linear distance (after depth and rotation are complete)
        dx = target_point.x - pose.pose.position.x
        dy = target_point.y - pose.pose.position.y
        linear_distance = (dx**2 + dy**2) ** 0.5
        linear_step_size = linear_distance / (time_per_phase * 10) if linear_distance > 0 else 0.1  # 10 Hz rate
        
        rospy.loginfo(f"Simulating movement to reach target in {time_to_reach} seconds")
        rospy.loginfo(f"Movement sequence: Depth → Rotation → Linear")

        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Update current phase based on progress
            if moving[0]:  # Depth adjustment
                dz = target_point.z - pose.pose.position.z
                depth_distance = abs(dz)
                
                if depth_distance > self.DELTA:
                    # Move up or down
                    pose.pose.position.z += math.copysign(depth_step_size, dz)
                    rospy.loginfo(f"Adjusting depth: current z={pose.pose.position.z:.2f}, target z={target_point.z:.2f}")
                else:
                    # Complete depth phase
                    rospy.loginfo("Depth adjustment complete")
                    moving = [False, True, False]  # Move to rotation phase
            
            elif moving[1]:  # Rotation adjustment
                # Calculate target yaw to face the target
                target_yaw = self.calculate_yaw_to_target(pose.pose.position, target_point)
                
                # Extract current yaw from quaternion
                current_orientation = pose.pose.orientation
                siny_cosp = 2 * (current_orientation.w * current_orientation.z + current_orientation.x * current_orientation.y)
                cosy_cosp = 1 - 2 * (current_orientation.y**2 + current_orientation.z**2)
                current_yaw = math.atan2(siny_cosp, cosy_cosp)
                
                # Calculate angle difference
                angle_diff = target_yaw - current_yaw
                while angle_diff > math.pi:
                    angle_diff -= 2.0 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2.0 * math.pi
                
                if abs(angle_diff) > self.DELTA:
                    # Rotate towards target
                    rotation_direction = 1 if angle_diff > 0 else -1
                    new_yaw = current_yaw + rotation_direction * rotation_step_size
                    pose.pose.orientation = self.euler_to_quaternion(new_yaw)
                    rospy.loginfo(f"Rotating: current={math.degrees(current_yaw):.1f}°, target={math.degrees(target_yaw):.1f}°")
                else:
                    # Complete rotation phase
                    rospy.loginfo("Rotation complete")
                    moving = [False, False, True]  # Move to linear phase
            
            elif moving[2]:  # Linear movement
                dx = target_point.x - pose.pose.position.x
                dy = target_point.y - pose.pose.position.y
                distance = (dx**2 + dy**2) ** 0.5
                
                if distance > self.DELTA:
                    # Move in a straight line towards target
                    if distance > 0:
                        pose.pose.position.x += linear_step_size * (dx / distance)
                        pose.pose.position.y += linear_step_size * (dy / distance)
                    rospy.loginfo(f"Moving forward: distance={distance:.2f}")
                else:
                    # Complete linear phase and entire movement
                    rospy.loginfo("Target reached")
                    break
            
            # Publish the updated pose
            self.pose_publisher.publish(pose)
            
            # Log current position
            rospy.loginfo(f"Current position: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, {pose.pose.position.z:.2f})")
            
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
