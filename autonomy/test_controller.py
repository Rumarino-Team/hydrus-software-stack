#!/usr/bin/env python3
import roslib
import arduino_simulator
from geometry_msgs.msg import Point
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointGoal

import sys
import unittest
import rospy
import time
import actionlib

class TestController(unittest.TestCase):
    client = None
    simulator = None
    DELTA = 0.20  # Threshold for considering a step complete
    PWM_NEUTRAL = 1500

    def setUp(self):
        self.client = actionlib.SimpleActionClient('controller_action', NavigateToWaypointAction)
        self.simulator = arduino_simulator.ArduinoSimulator()
    
    def tearDown(self):
        self.client = None
        self.simulator = None

    def run_simulation(self, target_point, time_to_reach, expected_neutral_thrusters=None):
        # Set up real-time thruster validation

        goal = NavigateToWaypointGoal()
        goal.target_point = target_point  # Assigning the Point object to the target_point field. Target aquiared. 

        self.client.send_goal(goal)

        rospy.loginfo("Waiting for result from the controller...")
        self.simulator.expected_neutral_thrusters = expected_neutral_thrusters
        self.client.wait_for_result(timeout=rospy.Duration(time_to_reach))
        self.simulator.expected_neutral_thrusters = None

        result = self.client.get_result()
        self.assertFalse(result is None, f"No result received from the controller action server, distance left = {self.simulator.target_distance}")

        rospy.loginfo(f"Final distance to target: {result.distance_to_target:.2f}")
        self.assertTrue(result.distance_to_target < 0.1, "Controller failed to reach the target within acceptable range.")
        rospy.loginfo("Controller successfully moved the submarine to the target.")
        return result
    
    def test_go_forward(self): # only functions with 'test_'-prefix will be run!
        target_point = Point()
        target_point.x = 2
        target_point.y = 0
        target_point.z = 0
        
        # For forward movement: depth thrusters (2, 7) and torpedo thrusters (3, 6) should stay neutral
        expected_neutral_thrusters = [2, 7, 3, 6]
        
        res = self.run_simulation(target_point, 30, expected_neutral_thrusters=expected_neutral_thrusters)

        self.assertTrue(res is not None, "Going forward failed!")
    
    def test_go_side(self): 
        target_point = Point()
        target_point.x = 0
        target_point.y = 2
        target_point.z = 0
        
        # For side movement: depth thrusters (2, 7) and torpedo thrusters (3, 6) should stay neutral
        expected_neutral_thrusters = [2, 7, 3, 6]
        
        res = self.run_simulation(target_point, 60, expected_neutral_thrusters=expected_neutral_thrusters)

        self.assertTrue(res is not None, "Going to the side failed!")

    def test_go_up(self):
        target_point = Point()
        target_point.x = 0
        target_point.y = 0
        target_point.z = 2
        
        # For upward movement: only torpedo thrusters (3, 6) should stay neutral
        # Depth thrusters (2, 7) will be active, horizontal thrusters (1, 4, 5, 8) may be used for positioning
        expected_neutral_thrusters = [3, 6]
        
        res = self.run_simulation(target_point, 30, expected_neutral_thrusters=expected_neutral_thrusters)
        
        self.assertTrue(res is not None, "Going up failed!")

    def test_go_point(self):
        target_point = Point()
        target_point.x = 2
        target_point.y = 2
        target_point.z = 2
        
        # For 3D movement: only torpedo thrusters (3, 6) should stay neutral
        # All other thrusters may be used for depth, rotation, and forward movement
        expected_neutral_thrusters = [3, 6]
        
        res = self.run_simulation(target_point, 30, expected_neutral_thrusters=expected_neutral_thrusters)
        
        self.assertTrue(res is not None, "Going to point failed!")
        
    def test_go_consecutive_points_x(self):
        target_point = Point()
        target_point.x = 2
        target_point.y = 0
        target_point.z = 0

        expected_neutral_thrusters = [2, 7, 3, 6]
        res = self.run_simulation(target_point, 30, expected_neutral_thrusters=expected_neutral_thrusters)
        self.assertTrue(res is not None, "Going to second point failed")

        target_point.x = 3
        target_point.y = 0
        target_point.z = 0

        res = self.run_simulation(target_point, 30, expected_neutral_thrusters=expected_neutral_thrusters)
        self.simulator = arduino_simulator.ArduinoSimulator()
        self.assertTrue(res is not None, "Going to second point failed")

    def test_go_consecutive_points_y(self):
        target_point = Point()
        target_point.x = 0
        target_point.y = 2
        target_point.z = 0

        expected_neutral_thrusters = [2, 7, 3, 6]
        res = self.run_simulation(target_point, 30, expected_neutral_thrusters=expected_neutral_thrusters)
        self.assertTrue(res is not None, "Going to second point failed")

        target_point.x = 0
        target_point.y = 3
        target_point.z = 0

        res = self.run_simulation(target_point, 30, expected_neutral_thrusters=expected_neutral_thrusters)
        self.simulator = arduino_simulator.ArduinoSimulator()
        self.assertTrue(res is not None, "Going to second point failed")
        
    def test_go_consecutive_points_z(self):
        target_point = Point()
        target_point.x = 0
        target_point.y = 0
        target_point.z = 2

        expected_neutral_thrusters = [2, 7, 3, 6]
        res = self.run_simulation(target_point, 30, expected_neutral_thrusters=expected_neutral_thrusters)
        self.assertTrue(res is not None, "Going to second point failed")

        target_point.x = 0
        target_point.y = 0
        target_point.z = 3

        res = self.run_simulation(target_point, 30, expected_neutral_thrusters=expected_neutral_thrusters)
        self.simulator = arduino_simulator.ArduinoSimulator()
        self.assertTrue(res is not None, "Going to second point failed")
if __name__ == '__main__':
    import rostest
    PKG = 'autonomy'
    rostest.rosrun(PKG, 'test_controller', TestController)