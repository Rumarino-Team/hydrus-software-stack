#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np
import math
#from driver_test import calculate_depth_from_beams
"""
I commented the import of the method from DVL because it 
causes an error when running it since the DVL is not connected. 
For that I temporarily moved the method that calculates the depth here.
"""

def calculate_depth_from_beams(beams, beam_angle_degrees=30):

    beam_angle_radians = math.radians(beam_angle_degrees)   
    depths = beams * np.cos(beam_angle_radians)
    
    depth = np.mean(depths)
    return depth
    
def dvl_callback(msg):
    beams = np.array(msg.data)
    depth = calculate_depth_from_beams(beams)
    rospy.loginfo(f"Aprox. Depth: {depth:.2f} m")
    depth_pub.publish(depth)

def dvl_listener():
    global depth_pub
    rospy.init_node('dvl_listener', anonymous=True)
    rospy.Subscriber('/dvl_data', Float32MultiArray, dvl_callback)
    depth_pub = rospy.Publisher('/dvl_depth', Float32, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    dvl_listener()
