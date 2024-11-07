#!/usr/bin/env python3

import math
import yaml
from controller.srv import NavigateToWaypoint, NavigateToWaypointResponse, SetParameters, SetParametersResponse
from controller.msg import NavigateToWaypointAction, NavigateToWaypointFeedback, NavigateToWaypointResult
import os
from types import point_3d, rotation_3d
import rospy
import actionlib

class SubController:
    def __init__(self):
        self.DEPTH_SPEED = 2
        self.ROTATION_SPEED = 1
        self.LINEAR_SPEED = 2
        self.FRONT_TORPEDO_SPEED = 5
        self.BACK_TORPEDO_SPEED = 4
        self.DELTA = 0.01

        self.speed_translation = {
            1: 1550,
            2: 1600,
            3: 1650,
            4: 1700,
            5: 1300,
            6: 1400,
            7: 1350
        }

        # ACSCII REPRESENTATION OF THE POSITION OF THE THUSTERS:
        #
        #  1 *            * 5
        #     \          /
        #      |________|        
        #  2*--|        |--* 6
        #      |        |
        #      |        |
        #  3*--|________|--* 7 
        #      |        |
        #  4 */          \*  8
        #    

        self.DEPTH_MOTORS_ID = [2, 7]  # 3 and 6 are not connected due to the Torpedos Connection
        self.FRONT_MOTORS_ID = [1, 5]  # left, right
        self.BACK_MOTORS_ID = [4, 8]  # left, right
        self.TORPEDO_MOTORS_ID = [3, 6]
        self.target_point = None
        self.thrusters_publishers = []
        self.thruster_values = [1500 for _ in range(8)]
        self.moving = [False, False, False]  # [depth, rotation, linear]
        for i in range(8):
            self.thrusters_publishers.append(rospy.Publisher('/hydrus/thrusters/' + str(i+1),  queue_size=10))


        self.server = actionlib.SimpleActionServer('navigate_to_waypoint', NavigateToWaypointAction, self.execute_callback, False)
        self.server.start()
        rospy.loginfo("NavigateToWaypoint Action Server started.")


    def execute_callback(self, goal):
        feedback = NavigateToWaypointFeedback()
        result = NavigateToWaypointResult()
        
        target_point = Pose()
        target_point.position.x = goal.x
        target_point.position.y = goal.y
        target_point.position.z = goal.z
        
        self.moving = [True, False, False] 
        current_pose = self.get_current_pose() 
        
        rate = rospy.Rate(10) 
        
        while not rospy.is_shutdown():
            self.move_submarine(current_pose, target_point)

            distance = self.calculate_distance(current_pose.position, target_point.position)
            feedback.distance_to_target = distance
            self.server.publish_feedback(feedback)
            
            if distance < self.DELTA:
                result.success = True
                self.server.set_succeeded(result)
                rospy.loginfo("Target reached.")
                return
            
            if self.server.is_preempt_requested():
                result.success = False
                self.server.set_preempted(result)
                rospy.loginfo("Goal preempted.")
                return
            
            current_pose = self.get_current_pose()
            rate.sleep()


    @staticmethod
    def move_submarine(self, current_pose, target_point):
        if current_pose is None or target_point is None:
            return
        if self.moving[0]:  # Move in the depth direction
            if abs(current_pose.position.z - target_point.position.z) > self.DELTA:
                self.adjust_depth_motors(current_pose, target_point)
            else:
                self.moving = [False, True, False]

        elif self.moving[1]:  # Rotate to face the target point
            if self.adjust_rotation_motors(current_pose, target_point):
                self.moving = [False, False, True]

        elif self.moving[2]:  # Move forward
            if self.adjust_linear_motors(current_pose, target_point):
                self.moving = [True, False, False]  # Reset to depth movement if necessary


        for i in range(len(self.thruster_values)):
            self.thrusters_publishers[i].publish(self.thruster_values[i])
        self.rate.sleep()

    def adjust_depth_motors(self, current_pose, target_point):
        # Adjust thrusters for depth
        if current_pose.position.z < target_point.position.z:
            for motor_id in self.DEPTH_MOTORS_ID:
                self.thruster_values[motor_id] = self.speed_translation[self.DEPTH_SPEED]
        else:
            for motor_id in self.DEPTH_MOTORS_ID:
                self.thruster_values[motor_id] = -self.speed_translation[self.DEPTH_SPEED]

    def adjust_rotation_motors(self, current_pose, target_point):
            # Adjust thrusters for rotation
            target_yaw = self.calculate_yaw_to_target(current_pose.position, target_point.position)
            current_yaw = self.calculate_current_yaw(current_pose.orientation)
            angle_diff = self.normalize_angle(target_yaw - current_yaw)
            
            if abs(angle_diff) > self.DELTA:
                if angle_diff > 0:
                    self.thruster_values[self.FRONT_MOTORS_ID[0]] = -self.speed_translation[self.ROTATION_SPEED]
                    self.thruster_values[self.FRONT_MOTORS_ID[1]] = self.speed_translation[self.ROTATION_SPEED]
                    self.thruster_values[self.BACK_MOTORS_ID[0]] = -self.speed_translation[self.ROTATION_SPEED]
                    self.thruster_values[self.BACK_MOTORS_ID[1]] = self.speed_translation[self.ROTATION_SPEED]
                else:
                    self.thruster_values[self.FRONT_MOTORS_ID[0]] = self.speed_translation[self.ROTATION_SPEED]
                    self.thruster_values[self.FRONT_MOTORS_ID[1]] = -self.speed_translation[self.ROTATION_SPEED]
                    self.thruster_values[self.BACK_MOTORS_ID[0]] = self.speed_translation[self.ROTATION_SPEED]
                    self.thruster_values[self.BACK_MOTORS_ID[1]] = -self.speed_translation[self.ROTATION_SPEED]
                return False
            else:
            # Target angle achieved
             return True

    @staticmethod
    def calculate_yaw_to_target(self, current_position, target_point):
        dx = target_point.x - current_position.x
        dy = target_point.y - current_position.y
        return math.atan2(dy, dx)
    @staticmethod
    def calculate_current_yaw(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)
    @staticmethod
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)





if __name__ == '__main__':
    try:
        controller = SubController()
    except rospy.ROSInterruptException:
        pass

