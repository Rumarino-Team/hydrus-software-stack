#!/usr/bin/env python3
import roslib
import arduino_simulator
from geometry_msgs.msg import Point

import sys
import unittest
import rospy
import time

class TestController(unittest.TestCase):
    simulator = arduino_simulator.ArduinoSimulator()
    DELTA = 0.20  # Threshold for considering a step complete
    PWM_NEUTRAL = 1500
    
    def test_go_forward(self): # only functions with 'test_'-prefix will be run!
        target_point = Point()
        target_point.x = 2
        target_point.y = 0
        target_point.z = 0
        
        # For forward movement: depth thrusters (2, 7) and torpedo thrusters (3, 6) should stay neutral
        expected_neutral_thrusters = [2, 7, 3, 6]
        
        res = self.simulator.test_controller(target_point, expected_neutral_thrusters=expected_neutral_thrusters)
        self.simulator = arduino_simulator.ArduinoSimulator()

        self.assertTrue(res is not None, "Going forward failed!")
    
    def test_go_side(self): 
        target_point = Point()
        target_point.x = 0
        target_point.y = 2
        target_point.z = 0
        
        # For side movement: depth thrusters (2, 7) and torpedo thrusters (3, 6) should stay neutral
        expected_neutral_thrusters = [2, 7, 3, 6]
        
        res = self.simulator.test_controller(target_point, expected_neutral_thrusters=expected_neutral_thrusters)
        self.simulator = arduino_simulator.ArduinoSimulator()

        self.assertTrue(res is not None, "Going to the side failed!")

    def test_go_up(self):
        target_point = Point()
        target_point.x = 0
        target_point.y = 0
        target_point.z = 2
        
        # For upward movement: only torpedo thrusters (3, 6) should stay neutral
        # Depth thrusters (2, 7) will be active, horizontal thrusters (1, 4, 5, 8) may be used for positioning
        expected_neutral_thrusters = [3, 6]
        
        res = self.simulator.test_controller(target_point, expected_neutral_thrusters=expected_neutral_thrusters)
        self.simulator = arduino_simulator.ArduinoSimulator()
        
        self.assertTrue(res is not None, "Going up failed!")

    def test_go_point(self):
        target_point = Point()
        target_point.x = 2
        target_point.y = 2
        target_point.z = 2
        
        # For 3D movement: only torpedo thrusters (3, 6) should stay neutral
        # All other thrusters may be used for depth, rotation, and forward movement
        expected_neutral_thrusters = [3, 6]
        
        res = self.simulator.test_controller(target_point, expected_neutral_thrusters=expected_neutral_thrusters)
        self.simulator = arduino_simulator.ArduinoSimulator()
        
        self.assertTrue(res is not None, "Going to point failed!")

if __name__ == '__main__':
    import rostest
    PKG = 'autonomy'
    rostest.rosrun(PKG, 'test_controller', TestController)