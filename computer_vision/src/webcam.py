#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

def find_available_cameras(max_cameras=10):
    """
    Check for available camera indices.
    """
    available_cameras = []
    for i in range(max_cameras):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(i)
        cap.release()
    return available_cameras

def webcam_publisher(camera_id):
    """
    Function to capture video from a specific camera and publish to a unique ROS topic.
    """
    rospy.init_node('multi_webcam_publisher', anonymous=True)

    # Unique topic for each camera
    topic_name = f'/camera_{camera_id}/image_raw'
    image_pub = rospy.Publisher(topic_name, Image, queue_size=10)
    
    cap = cv2.VideoCapture(camera_id)
    
    if not cap.isOpened():
        rospy.logerr(f"Unable to open webcam {camera_id}")
        return

    rate = rospy.Rate(10)  # 10 Hz
    bridge = CvBridge()

    rospy.loginfo(f"Publishing camera {camera_id} on topic {topic_name}")
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr(f"Failed to capture image from camera {camera_id}")
            continue
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_pub.publish(image_message)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        available_cameras = find_available_cameras()

        if not available_cameras:
            rospy.logerr("No cameras found!")
        else:
            rospy.loginfo(f"Available cameras: {available_cameras}")

            # Start a separate thread for each camera
            threads = []
            for camera_id in available_cameras:
                thread = threading.Thread(target=webcam_publisher, args=(camera_id,))
                thread.start()
                threads.append(thread)

            # Wait for all threads to complete
            for thread in threads:
                thread.join()

    except rospy.ROSInterruptException:
        pass
