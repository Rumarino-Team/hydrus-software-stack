#!/usr/bin/env python3

import rospy
import argparse
from computer_vision.srv import EnableDetector, EnableDetectorRequest

def enable_detector(detector_name):
    rospy.wait_for_service('enable_detector')
    try:
        enable_detector_service = rospy.ServiceProxy('enable_detector', EnableDetector)
        response = enable_detector_service(EnableDetectorRequest(detector_name=detector_name))
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

if __name__ == '__main__':
    rospy.init_node('enable_detector_cli')

    # Set up argument parser
    parser = argparse.ArgumentParser(description='Enable a specific detector')
    parser.add_argument('--detector', type=str, help='Name of the detector to enable (e.g., "yolo" or "another_model")', default= "yolo_world")

    args = parser.parse_args()
    
    # Enable the specified dete ctor
    success = enable_detector(args.detector)
    if success:
        rospy.loginfo(f"{args.detector} enabled successfully.")
    else:
        rospy.logerr(f"Failed to enable {args.detector}.")

    rospy.spin()
