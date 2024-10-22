#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def webcam_publisher():
    rospy.init_node('webcam_publisher', anonymous=True)
    
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        rospy.logerr("Unable to open the webcam")
        return
    
    rate = rospy.Rate(10) 
    
    bridge = CvBridge()
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture image")
            continue
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_pub.publish(image_message)
        rate.sleep()
    cap.release()

if __name__ == '__main__':
    try:
        webcam_publisher()
    except rospy.ROSInterruptException:
        pass
