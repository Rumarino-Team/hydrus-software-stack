#!/usr/bin/env python3

import rospy
import argparse
from computer_vision.srv import SetCustomClasses, SetCustomClassesRequest

def set_custom_classes(classes, save_path):
    rospy.wait_for_service('set_custom_classes')
    try:
        set_custom_classes_service = rospy.ServiceProxy('set_custom_classes', SetCustomClasses)
        response = set_custom_classes_service(SetCustomClassesRequest(classes=classes, save_path=save_path))
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

if __name__ == '__main__':
    rospy.init_node('set_custom_classes_cli')

    parser = argparse.ArgumentParser(description='Set custom classes and save YOLO model')
    parser.add_argument('--classes', nargs='+', required=True, help='List of custom classes')

    args = parser.parse_args()
    
    success = set_custom_classes(args.classes, args.save_path)
    if success:
        rospy.loginfo(f"Custom classes set and model saved to {args.save_path}.")
    else:
        rospy.logerr("Failed to set custom classes and save the model.")

    rospy.spin()
