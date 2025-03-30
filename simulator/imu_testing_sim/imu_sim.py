#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import numpy as np

class OdomSimulator:
    def __init__(self):
        rospy.init_node('odom_simulator', anonymous=True)
        """
        Still not tested. Having problems downloding new version of Gazebo. 
        """
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.rate = rospy.Rate(50)  # 50Hz (If there is problems try 10Hz)
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.run()

    def odom_callback(self, msg):
        """ Callback function for Odometry subscriber """
        rospy.loginfo(f"Received Odom Data: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}, z={msg.pose.pose.position.z}")

    def publish_odom_data(self):
        """ Publishes simulated odometry data """
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'

        self.x += np.random.uniform(-0.1, 0.1)
        self.y += np.random.uniform(-0.1, 0.1)
        self.z += np.random.uniform(-0.05, 0.05)

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = self.z

        odom_msg.pose.pose.orientation.w = 1.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0

        self.pub.publish(odom_msg)
        rospy.loginfo(f"Published Odom Data: x={self.x}, y={self.y}, z={self.z}")

    def run(self):
        """ Runs the node, publishing data at a fixed rate (50Hz)"""
        while not rospy.is_shutdown():
            self.publish_odom_data()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        OdomSimulator()
    except rospy.ROSInterruptException:
        pass
