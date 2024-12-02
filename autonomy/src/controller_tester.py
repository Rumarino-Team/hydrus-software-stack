#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, PoseStamped
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointGoal
import actionlib
import random
import time


class ControllerTester:
    def __init__(self):
        rospy.init_node('controller_tester', anonymous=True)

        self.pose_publisher = rospy.Publisher('/zed2i/zed_node/pose', PoseStamped, queue_size=10)
        self.client = actionlib.SimpleActionClient('controller_action', NavigateToWaypointAction)

        rospy.loginfo("Waiting for the controller action server to start...")
        self.client.wait_for_server()
        rospy.loginfo("Controller action server started.")

    def simulate_pose(self, target_point):
        """
        Simulates the submarine's movement towards a target point by publishing updated positions.
        """
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            step_size = 0.1
            dx = target_point.x - pose.pose.position.x
            dy = target_point.y - pose.pose.position.y
            dz = target_point.z - pose.pose.position.z

            distance = (dx**2 + dy**2 + dz**2) ** 0.5
            if distance < 0.1:  
                rospy.loginfo("Submarine reached the target point.")
                break

            pose.pose.position.x += step_size * (dx / distance)
            pose.pose.position.y += step_size * (dy / distance)
            pose.pose.position.z += step_size * (dz / distance)

            self.pose_publisher.publish(pose)

            rospy.loginfo(f"Current position: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, {pose.pose.position.z:.2f})")

            rate.sleep()

    def test_controller(self, target_point):
        """
        Tests the controller by sending a target point and simulating movement.
        """
        rospy.loginfo(f"Testing controller with target point: ({target_point.x}, {target_point.y}, {target_point.z})")

        goal = NavigateToWaypointGoal()
        goal.target_point = target_point  # Assigning the Point object to the target_point field. Target aquiared. 

        self.client.send_goal(goal)

        rospy.loginfo("Simulating submarine pose updates...")
        self.simulate_pose(target_point)

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

    def run(self):
        """
        Runs the tester with a predefined target point.
        """
        #Chooses a random target because testing:)
        target_point = Point()
        target_point.x = random.uniform(1.0, 5.0)
        target_point.y = random.uniform(1.0, 5.0)
        target_point.z = random.uniform(1.0, 3.0)

        self.test_controller(target_point)


if __name__ == '__main__':
    try:
        tester = ControllerTester()
        tester.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller tester interrupted.")
