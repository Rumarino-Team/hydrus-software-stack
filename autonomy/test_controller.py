#!/usr/bin/env python3
PKG = 'autonomy'
import roslib
import arduino_simulator
from geometry_msgs.msg import Point

import sys
import unittest

class TestController(unittest.TestCase):
    simulator = arduino_simulator.ArduinoSimulator()
    DELTA = 0.20  # Threshold for considering a step complete

    def test_go_forward(self): # only functions with 'test_'-prefix will be run!
        target_point = Point()
        target_point.x = 2
        target_point.y = 0
        target_point.z = 0
        res = self.simulator.test_controller(target_point)
        self.simulator = arduino_simulator.ArduinoSimulator()

        self.assertTrue(res, "Going forward failed!")
    
    def test_go_side(self): 
        target_point = Point()
        target_point.x = 0
        target_point.y = 2
        target_point.z = 0
        res = self.simulator.test_controller(target_point)
        self.simulator = arduino_simulator.ArduinoSimulator()

        self.assertTrue(res, "Going to the side failed!")

    def test_go_up(self):
        target_point = Point()
        target_point.x = 0
        target_point.y = 0
        target_point.z = 2
        res = self.simulator.test_controller(target_point)
        self.simulator = arduino_simulator.ArduinoSimulator()
        self.assertTrue(res, "Going up failed!")

    def test_go_point(self):
        target_point = Point()
        target_point.x = 2
        target_point.y = 2
        target_point.z = 2
        res = self.simulator.test_controller(target_point)
        self.simulator = arduino_simulator.ArduinoSimulator()
        self.assertTrue(res, "Going to point failed!")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_controller', TestController)