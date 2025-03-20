#!/usr/bin/env python3

import math
import rospy
import actionlib
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32
from dataclasses import dataclass, field
from typing import List
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointFeedback, NavigateToWaypointResult



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
        DEPTH_SPEED: int = 1
        ROTATION_SPEED: int = 1
        LINEAR_SPEED: int = 1
        FRONT_TORPEDO_SPEED: int = 5
        BACK_TORPEDO_SPEED: int = 4
        DELTA: float = 0.01
        SPEED_TRANSLATION: dict = field(default_factory=lambda: {
            1: 1550,
            2: 1600,
            3: 1650,
            4: 1700,
            5: 1300,
            6: 1400,
            7: 1350
        })
        TOTAL_THRUSTERS: int = 8
        DEPTH_MOTORS_ID: list = field(default_factory=lambda: [2, 7])
        FRONT_MOTORS_ID: list = field(default_factory=lambda: [1, 5])
        BACK_MOTORS_ID: list = field(default_factory=lambda: [4, 8])
        TORPEDO_MOTORS_ID: list = field(default_factory=lambda: [3, 6])

    def __init__(self):
        # ////////////////////////////////////
        # ////////ROS MUTABLE OBJECTS/////////
        # ////////////////////////////////////
        
        self.thrusters_publishers = []
        self.server = None
        self.target_point = None
        self.submarine_pose = None
        self.thruster_values = [1500 for _ in range(8)]
        rospy.loginfo(f"Runnin?({self.thruster_values})")
        self.moving: List[bool] = [False, False, False]  # [depth, rotation, linear]

        #//////////////////////////////////// 
        #////////// INIT ROS DATA////////////
        #////////////////////////////////////
        for i in range(self.Constants.TOTAL_THRUSTERS):
            self.thrusters_publishers.append(rospy.Publisher('/thrusters/' + str(i+1), Float32, queue_size=10))
        def imu_pose_callback(msg):
            self.submarine_pose = msg
        rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, imu_pose_callback)
        self.server = actionlib.SimpleActionServer('controller_action', NavigateToWaypointAction, self.execute_callback, False)
        self.server.start()
        

    def execute_callback(self, goal): 
        feedback = NavigateToWaypointFeedback()
        result = NavigateToWaypointResult()

        # The target point is set
        target_point = goal.target_point
        rospy.loginfo(f"Goal received: target_point=({target_point.x}, {target_point.y}, {target_point.z})")

        rate = rospy.Rate(10)  # This is set to 10 hz but you can change it if you like

        while not rospy.is_shutdown():
            if self.submarine_pose is None:
                rospy.logwarn("Submarine pose is not available yet.")
                rate.sleep()
                continue

            # Update feedback
            distance = self.calculate_distance(self.submarine_pose.pose.position, target_point)
            feedback.success = distance < self.Constants.DELTA  # Update feedback so you know how close it is
            rospy.loginfo(f"Distance to target: {distance:.2f}")

            # Publish feedback
            self.server.publish_feedback(feedback)

            # Check if the target is reached
            if distance - self.Constants.DELTA<0.05:
                rospy.loginfo("Target reached.")
                result.distance_to_target = distance
                self.server.set_succeeded(result)  # Goal is reached, YAY!
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

        position = current_pose.pose.position

        if self.moving[0]:  # Move in the depth direction
            if abs(current_pose.position.z - target_point.position.z) > self.Constants.DELTA:
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
        rospy.loginfo(f"Current position: ({current_pose.pose.position.x}, {current_pose.pose.position.y}, {current_pose.pose.position.z})")



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

