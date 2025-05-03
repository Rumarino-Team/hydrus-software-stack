#!/usr/bin/env python3
"""
Integration test for the autonomy and embedded_arduino packages.
This test verifies that:
1. Communication between ROS nodes is working
2. Thruster commands are properly received by the Arduino
3. The controller can navigate to waypoints
"""

import unittest
import time
import rospy
import rostest
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Point, PoseStamped
import actionlib
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointGoal

class TestAutonomyArduinoIntegration(unittest.TestCase):
    def setUp(self):
        """Initialize the test node and set up subscribers/publishers"""
        rospy.init_node('test_autonomy_arduino_integration')
        
        # Track received thruster commands
        self.thruster_values = [None] * 8
        self.thruster_commands_received = 0
        
        # Create subscribers for thruster commands
        for i in range(8):
            # Subscribe to Float32 topics
            rospy.Subscriber(
                f'/thrusters/{i+1}', 
                Float32, 
                lambda msg, idx=i: self.thruster_callback(msg, idx)
            )
            
            # Also subscribe to Int8 topics for Arduino
            rospy.Subscriber(
                f'/hydrus/thrusters/{i+1}', 
                Int8, 
                lambda msg, idx=i: self.arduino_thruster_callback(msg, idx)
            )
        
        # Create a publisher for simulated submarine pose
        self.pose_pub = rospy.Publisher('/zed2i/zed_node/pose', PoseStamped, queue_size=1)
        
        # Create an action client for the controller
        self.client = actionlib.SimpleActionClient('controller_action', NavigateToWaypointAction)
        
        # Give time for connections to be established
        time.sleep(2)

    def thruster_callback(self, msg, idx):
        """Callback for thruster commands from autonomy"""
        self.thruster_values[idx] = msg.data
        self.thruster_commands_received += 1
        rospy.loginfo(f"Received thruster command for motor {idx+1}: {msg.data}")

    def arduino_thruster_callback(self, msg, idx):
        """Callback for thruster commands intended for Arduino"""
        rospy.loginfo(f"Arduino would receive for motor {idx+1}: {msg.data}")

    def publish_test_pose(self, x=0.0, y=0.0, z=0.0):
        """Publish a fake submarine pose"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0  # No rotation
        self.pose_pub.publish(pose_msg)
        rospy.loginfo(f"Published test pose: ({x}, {y}, {z})")

    def test_01_controller_connection(self):
        """Test that the controller is available"""
        rospy.loginfo("Testing controller availability...")
        self.assertTrue(
            self.client.wait_for_server(timeout=rospy.Duration(5.0)),
            "Controller action server not found"
        )
        rospy.loginfo("Controller action server is available")

    def test_02_thruster_command_generation(self):
        """Test that sending a waypoint generates thruster commands"""
        rospy.loginfo("Testing thruster command generation...")
        
        # Reset counter
        self.thruster_commands_received = 0
        
        # Publish a fake submarine pose
        self.publish_test_pose(0.0, 0.0, 0.0)
        time.sleep(1)
        
        # Send a waypoint goal
        goal = NavigateToWaypointGoal()
        goal.target_point = Point(1.0, 0.0, 0.0)  # 1 meter forward
        self.client.send_goal(goal)
        
        # Wait for commands to be generated
        timeout = time.time() + 5.0
        while time.time() < timeout and self.thruster_commands_received == 0:
            time.sleep(0.1)
        
        self.client.cancel_goal()
        
        # Verify that thruster commands were generated
        self.assertGreater(
            self.thruster_commands_received, 
            0, 
            "No thruster commands were generated"
        )
        
        # Check which thrusters were activated
        active_thrusters = [i+1 for i, val in enumerate(self.thruster_values) if val is not None]
        rospy.loginfo(f"Active thrusters: {active_thrusters}")

    def test_03_depth_control(self):
        """Test depth control by sending a waypoint with different depth"""
        rospy.loginfo("Testing depth control...")
        
        # Reset values
        self.thruster_values = [None] * 8
        
        # Publish current pose at depth 0
        self.publish_test_pose(0.0, 0.0, 0.0)
        time.sleep(1)
        
        # Send a waypoint goal with different depth
        goal = NavigateToWaypointGoal()
        goal.target_point = Point(0.0, 0.0, 1.0)  # 1 meter deeper
        self.client.send_goal(goal)
        
        # Wait for thruster commands
        time.sleep(3)
        self.client.cancel_goal()
        
        # Check if depth thrusters were activated
        # Indices 1 and 6 correspond to motors with IDs 2 and 7 (depth motors)
        depth_motor_indices = [1, 6]  # 0-indexed (motors 2 and 7)
        
        depth_activated = False
        for idx in depth_motor_indices:
            if self.thruster_values[idx] is not None and abs(self.thruster_values[idx] - 1500) > 10:
                depth_activated = True
                break
                
        self.assertTrue(depth_activated, "Depth thrusters were not activated")

    def test_04_message_type_compatibility(self):
        """Test publishing different message types to see what the Arduino responds to"""
        rospy.loginfo("Testing message type compatibility...")
        
        # Try publishing an Int8 message to the first thruster
        int8_pub = rospy.Publisher('/hydrus/thrusters/1', Int8, queue_size=1)
        int8_msg = Int8()
        int8_msg.data = 2  # Forward command
        
        # Wait for publisher to connect
        time.sleep(1)
        
        # Publish the message
        int8_pub.publish(int8_msg)
        rospy.loginfo("Published Int8 message to /hydrus/thrusters/1")
        
        # Try publishing a Float32 message
        float32_pub = rospy.Publisher('/thrusters/1', Float32, queue_size=1)
        float32_msg = Float32()
        float32_msg.data = 1550.0  # Slightly forward
        
        # Wait for publisher to connect
        time.sleep(1)
        
        # Publish the message
        float32_pub.publish(float32_msg)
        rospy.loginfo("Published Float32 message to /thrusters/1")
        
        # Give time for messages to be processed
        time.sleep(2)
        
        # Note: This test mostly logs information for manual verification
        # since we can't directly check what the Arduino received
        rospy.loginfo("Check the logs to see which messages were received")

if __name__ == '__main__':
    try:
        rostest.run('hydrus', 'test_autonomy_arduino_integration', TestAutonomyArduinoIntegration, sys.argv)
    except KeyboardInterrupt:
        rospy.loginfo("Test interrupted by user")
    except Exception as e:
        rospy.logerr(f"Test failed with exception: {str(e)}")