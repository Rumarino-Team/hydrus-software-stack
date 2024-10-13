#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3

class SubListener:
    def __init__(self):
        rospy.init_node('sub_listener', anonymous=True)

        # Subscribers to the same topics published by SubController
        rospy.Subscriber('/hydrus/thrusters/1', Vector3, self.thruster_callback_1)
        rospy.Subscriber('/hydrus/thrusters/2', Vector3, self.thruster_callback_2)
        rospy.Subscriber('/hydrus/thrusters/3', Vector3, self.thruster_callback_3)
        rospy.Subscriber('/hydrus/thrusters/4', Vector3, self.thruster_callback_4)
        rospy.Subscriber('/hydrus/thrusters/5', Vector3, self.thruster_callback_5)
        rospy.Subscriber('/hydrus/thrusters/6', Vector3, self.thruster_callback_6)
        rospy.Subscriber('/hydrus/thrusters/7', Vector3, self.thruster_callback_7)
        rospy.Subscriber('/hydrus/thrusters/8', Vector3, self.thruster_callback_8)
        
        rospy.spin()  # Keep the subscriber running

    def thruster_callback_1(self, msg):
        rospy.loginfo("Thruster 1 command received: %s", msg)
    
    def thruster_callback_2(self, msg):
        rospy.loginfo("Thruster 2 command received: %s", msg)
    
    def thruster_callback_3(self, msg):
        rospy.loginfo("Thruster 3 command received: %s", msg)
    
    def thruster_callback_4(self, msg):
        rospy.loginfo("Thruster 4 command received: %s", msg)
    
    def thruster_callback_5(self, msg):
        rospy.loginfo("Thruster 5 command received: %s", msg)
    
    def thruster_callback_6(self, msg):
        rospy.loginfo("Thruster 6 command received: %s", msg)
    
    def thruster_callback_7(self, msg):
        rospy.loginfo("Thruster 7 command received: %s", msg)
    
    def thruster_callback_8(self, msg):
        rospy.loginfo("Thruster 8 command received: %s", msg)

if __name__ == '__main__':
    try:
        listener = SubListener()
    except rospy.ROSInterruptException:
        pass
