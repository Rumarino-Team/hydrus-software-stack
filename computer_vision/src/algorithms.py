#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32

def color_filter_callback(ros_image, args):
    tolerance, min_area, min_confidence, pub_filtered_image, pub_object_count = args

    # Convert the ROS image message to a format OpenCV understands
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except Exception as e:
        rospy.logerr("Failed to convert image: %s" % str(e))
        return

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

    # Masks for color red
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10 + tolerance, 255, 255])
    lower_red2 = np.array([170 - tolerance, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask_red1 | mask_red2

    # Reduce noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    filtered_frame = cv2.bitwise_and(frame, frame, mask=mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    object_count = 0

    # Bounding boxes
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            object_count += 1
            x, y, w, h = cv2.boundingRect(contour)

            # Confidence level
            contour_mask = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)

            red_pixels_in_contour = cv2.bitwise_and(mask, mask, mask=contour_mask)
            red_pixel_count = np.sum(red_pixels_in_contour == 255)
            total_pixels_in_contour = w * h

            # To avoid dividing by 0
            if total_pixels_in_contour > 0:
                confidence = (red_pixel_count / total_pixels_in_contour) * 100
            else:
                confidence = 0

            # To avoid a percentage higher than 100%
            if confidence > 100:
                confidence = 100

            # Ignoring objects with low confidence levels
            if confidence > min_confidence:
                cv2.rectangle(filtered_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(filtered_frame, f"Conf: {confidence:.2f}%", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Publish number of objects detected
    pub_object_count.publish(object_count)

    # Convert the filtered frame back to a ROS Image message and publish
    try:
        filtered_image_msg = bridge.cv2_to_imgmsg(filtered_frame, "bgr8")
        pub_filtered_image.publish(filtered_image_msg)
    except Exception as e:
        rospy.logerr("Failed to convert filtered frame: %s" % str(e))

def color_filter_node():
    rospy.init_node('color_filter_node', anonymous=True)

    # Parameters with default values
    tolerance = rospy.get_param('~tolerance', 10)
    min_area = rospy.get_param('~min_area', 500)
    min_confidence = rospy.get_param('~min_confidence', 30)

    # Create a publisher for the filtered image and object count
    pub_filtered_image = rospy.Publisher('/color_filter/filtered_image', Image, queue_size=10)
    pub_object_count = rospy.Publisher('/color_filter/object_count', Int32, queue_size=10)

    # Subscribe to the camera topic
    rospy.Subscriber('/camera/image_raw', Image, color_filter_callback, 
                     (tolerance, min_area, min_confidence, pub_filtered_image, pub_object_count))

    rospy.loginfo("Color filter node started.")
    rospy.spin()

if __name__ == '__main__':
    try:
        color_filter_node()
    except rospy.ROSInterruptException:
        pass
