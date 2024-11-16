#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped, Point
from controller.srv import NavigateToWaypoint
from std_msgs.msg import Bool

class TestNode:
    def __init__(self):
        rospy.init_node('test_node', anonymous=True)

        # Create a client to call the NavigateToWaypoint service
        rospy.wait_for_service('navigate_to_waypoint')
        self.navigate_to_waypoint = rospy.ServiceProxy('navigate_to_waypoint', NavigateToWaypoint)

        # Subscribe to the position topic to get real feedback
        self.current_position = None
        rospy.Subscriber('/zed2i/zed_node/pose', PoseStamped, self.position_callback)

        # Parameters for testing
        self.tolerance = 0.05  # 5 cm tolerance
        self.rate = rospy.Rate(10)  # 10 Hz update rate for test node

    def position_callback(self, msg):
        """Callback to update the current position from the actual node feedback."""
        self.current_position = msg.pose.position

    def send_target(self, x, y, z):
        """Send a target point to the controller and log the result of the navigation service call."""
        target_point = Point(x, y, z)
        try:
            response = self.navigate_to_waypoint(target_point)
            if response.success:
                rospy.loginfo(f"Target waypoint ({x}, {y}, {z}) set successfully.")
            else:
                rospy.logwarn("Failed to set target waypoint.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def has_reached_target(self, target_point):
        """Check if the current position is within tolerance of the target point."""
        if self.current_position is None:
            return False
        distance = math.sqrt(
            (self.current_position.x - target_point.x) ** 2 +
            (self.current_position.y - target_point.y) ** 2 +
            (self.current_position.z - target_point.z) ** 2
        )
        return distance <= self.tolerance

    def print_current_location(self):
        """Print the current location of the submarine."""
        if self.current_position is not None:
            rospy.loginfo(
                f"Current location of the submarine: "
                f"x={self.current_position.x:.2f}, "
                f"y={self.current_position.y:.2f}, "
                f"z={self.current_position.z:.2f}"
            )
        else:
            rospy.loginfo("Current location is not available yet.")

    def run_tests(self):
        # Define test cases for movement in different directions
        test_cases = [
            ("Forward", (5, 0, 0)),
            ("Upward", (0, 0, 5)),
            ("Downward", (0, 0, -5)),
            ("Diagonal", (5, 5, 5))
        ]

        for name, (x, y, z) in test_cases:
            rospy.loginfo(f"Running test case: {name}")
            target_point = Point(x, y, z)
            self.send_target(x, y, z)

            # Track progress to the target with a timeout
            reached_target = False
            start_time = rospy.Time.now()
            timeout = rospy.Duration(30)  # 30-second timeout for each target

            while not reached_target and not rospy.is_shutdown():
                if rospy.Time.now() - start_time > timeout:
                    rospy.logwarn(f"{name} movement test timed out without reaching the target.")
                    break

                # Check if we have reached the target within tolerance
                reached_target = self.has_reached_target(target_point)
                if reached_target:
                    rospy.loginfo(f"{name} movement test completed successfully. Target reached.")
                else:
                    # Log distance to target if still moving
                    if self.current_position:
                        distance = math.sqrt(
                            (self.current_position.x - target_point.x) ** 2 +
                            (self.current_position.y - target_point.y) ** 2 +
                            (self.current_position.z - target_point.z) ** 2
                        )
                        rospy.loginfo(f"{name} test in progress. Current distance to target: {distance:.2f} meters.")
                    # Call print_current_location to display current location
                    self.print_current_location()
                rospy.sleep(1)

if __name__ == '__main__':
    try:
        test_node = TestNode()
        test_node.run_tests()
    except rospy.ROSInterruptException:
        pass
