#!/usr/bin/env python

import rospy
from computer_vision.msg import Detection, Detections

def detection_callback(msg):
    rospy.loginfo("Received detections:")
    for detection, class_name in zip(msg.detections, msg.class_names):
        rospy.loginfo("Class: %s", class_name)
        rospy.loginfo("Confidence: %.2f", detection.confidence)
        rospy.loginfo("Bounding Box: x_offset=%d, y_offset=%d, height=%d, width=%d",
                      detection.bounding_box.x_offset, detection.bounding_box.y_offset,
                      detection.bounding_box.height, detection.bounding_box.width)
        rospy.loginfo("3D Point: x=%.2f, y=%.2f, z=%.2f",
                      detection.point.x, detection.point.y, detection.point.z)

def detection_listener():
    rospy.init_node('detection_listener', anonymous=True)
    rospy.Subscriber('/detector/box_detection', Detections, detection_callback)
    rospy.spin()

if __name__ == '__main__':
    detection_listener()
