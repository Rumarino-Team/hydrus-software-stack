#!/usr/bin/env python3

import math
import rospy
import actionlib
from geometry_msgs.msg import Point, PoseStamped
from autonomy.srv import NavigateToWaypoint, NavigateToWaypointResponse, SetParameters, SetParametersResponse
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointFeedback, NavigateToWaypointResult
from dataclasses import dataclass
from typing import List


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
        DEPTH_SPEED: int = 2
        ROTATION_SPEED: int = 1
        LINEAR_SPEED: int = 2
        FRONT_TORPEDO_SPEED: int = 5
        BACK_TORPEDO_SPEED: int = 4
        DELTA: float = 0.01
        SPEED_TRANSLATION: dict = {
            1: 1550,
            2: 1600,
            3: 1650,
            4: 1700,
            5: 1300,
            6: 1400,
            7: 1350
        }
        TOTAL_THRUSTERS: int = 8
        DEPTH_MOTORS_ID: list = [2, 7]
        FRONT_MOTORS_ID: list = [1, 5]
        BACK_MOTORS_ID: list = [4, 8]
        TORPEDO_MOTORS_ID: list = [3, 6]

    def __init__(self):
        # ////////////////////////////////////
        # ////////ROS MUTABLE OBJECTS/////////
        # ////////////////////////////////////
        self.thrusters_publishers = []
        self.server = None
        self.target_point = None
        self.submarine_pose = None
        self.thruster_values = [1500 for _ in range(8)]
        self.moving: List[bool] = [False, False, False]  # [depth, rotation, linear]

        #//////////////////////////////////// 
        #////////// INIT ROS DATA////////////
        #////////////////////////////////////
        for i in range(self.Constants.TOTAL_THRUSTERS):
            self.thrusters_publishers.append(rospy.Publisher('/thrusters/' + str(i+1), queue_size=10))
        def imu_pose_callback(msg):
            self.submarine_pose = msg
        rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, imu_pose_callback)


    def execute_callback(self,goal):

        feedback = NavigateToWaypointFeedback()
        result = NavigateToWaypointResult()

        # Set target point
        target_point = Point()
        target_point.x = goal.x
        target_point.position.y = goal.y
        target_point.position.z = goal.z
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.move_submarine(self.submarine_pose, target_point)
            distance = self.calculate_distance(self.submarine_pose.position, target_point.position)
            feedback.distance_to_target = distance
            self.server.publish_feedback(feedback)

            if distance < self.Constant.DELTA:
                result.success = True
                self.server.set_succeeded(result)
                rospy.loginfo("Target reached.")
                return

            if self.server.is_preempt_requested():
                result.success = False
                self.server.set_preempted(result)
                rospy.loginfo("Goal preempted.")
                return
            rate.sleep()


    def move_submarine(self,current_pose, target_point):
        if current_pose is None or target_point is None:
            return

        if moving[0]:  # Move in the depth direction
            if abs(current_pose.position.z - target_point.position.z) > self.Constants.DELTA:
                self.adjust_depth_motors(current_pose, target_point)
            else:
                moving = [False, True, False]

        elif moving[1]:  # Rotate to face the target point
            if self.adjust_rotation_motors(current_pose, target_point):
                moving = [False, False, True]

        elif moving[2]:  # Move forward
            if self.adjust_linear_motors(current_pose, target_point):
                moving = [True, False, False]  # Reset to depth movement if necessary

        for i in range(len(self.thruster_values)):
            self.thrusters_publishers[i].publish(self.thruster_values[i])


    def adjust_depth_motors(self, current_pose, target_point):
            if current_pose.position.z < target_point.position.z:
                for motor_id in self.Constants.DEPTH_MOTORS_ID:
                    self.thruster_values[motor_id] = self.Constants.speed_translation[self.Constants.DEPTH_SPEED]
            else:
                for motor_id in self.Constants.DEPTH_MOTORS_ID:
                    self.thruster_values[motor_id] = -self.Constants.speed_translation[self.Constants.DEPTH_SPEED]

    def adjust_rotation_motors(self, current_pose, target_point):
            target_yaw = self.calculate_yaw_to_target(current_pose.position, target_point.position)
            current_yaw = self.calculate_current_yaw(current_pose.orientation)
            angle_diff = self.normalize_angle(target_yaw - current_yaw)
            
            if abs(angle_diff) > self.Constants.DELTA:
                if angle_diff > 0:
                    self.thruster_values[self.FRONT_MOTORS_ID[0]] = -self.Constants.speed_translation[self.ROTATION_SPEED]
                    self.thruster_values[self.FRONT_MOTORS_ID[1]] = self.Constants.speed_translation[self.ROTATION_SPEED]
                    self.thruster_values[self.BACK_MOTORS_ID[0]] = -self.Constants.speed_translation[self.ROTATION_SPEED]
                    self.thruster_values[self.BACK_MOTORS_ID[1]] = self.Constants.speed_translation[self.ROTATION_SPEED]
                else:
                    self.thruster_values[self.FRONT_MOTORS_ID[0]] = self.Constants.speed_translation[self.ROTATION_SPEED]
                    self.thruster_values[self.FRONT_MOTORS_ID[1]] = -self.Constants.speed_translation[self.ROTATION_SPEED]
                    self.thruster_values[self.BACK_MOTORS_ID[0]] = self.Constants.speed_translation[self.ROTATION_SPEED]
                    self.thruster_values[self.BACK_MOTORS_ID[1]] = -self.Constants.speed_translation[self.ROTATION_SPEED]
                return False
            else:
                return True
            

    def adjust_linear_motors(self, current_pose, target_point):
            dx = target_point.position.x - current_pose.position.x
            dy = target_point.position.y - current_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)

            if distance > self.Constants.DELTA:
                for motor_id in self.Constants.FRONT_MOTORS_ID:
                    self.thruster_values[motor_id] = self.Constants.speed_translation[self.LINEAR_SPEED]
                for motor_id in self.BACK_MOTORS_ID:
                    self.thruster_values[motor_id] = self.Constants.speed_translation[self.LINEAR_SPEED]
                return False  
            else:
                # Stop forward movement
                for motor_id in self.Constants.FRONT_MOTORS_ID:
                    self.thruster_values[motor_id] = 1500  # Set to neutral thrust
                for motor_id in self.Constants.BACK_MOTORS_ID:
                    self.thruster_values[motor_id] = 1500  # Set to neutral thrust
                return True  # Indicate that the target has been reached

    @staticmethod
    def calculate_yaw_to_target(current_position, target_position):
        dx = target_position.x - current_position.x
        dy = target_position.y - current_position.y
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