#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Float32MultiArray

def dvl_publisher():
    pub = rospy.Publisher('/dvl_data', Float32MultiArray, queue_size=10)
    rospy.init_node('dvl_publisher', anonymous=True)
    rate = rospy.Rate(1)  
    
    testing_measures = [[10.0, 10.2, 9.8, 10.1],[9.5,8,10,2],[10.2,9.8,9.7],[9.9,10.0,9.5]] 
    while not rospy.is_shutdown():
        simulated_data = Float32MultiArray()
        simulated_data.data = random.choice(testing_measures)
        pub.publish(simulated_data)
        rospy.loginfo("Sending Dvl test data: %s", simulated_data.data)
        rate.sleep()

if __name__ == '__main__':
    dvl_publisher()
