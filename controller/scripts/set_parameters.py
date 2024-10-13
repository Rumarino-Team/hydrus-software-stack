#!/usr/bin/env python

import rospy
from controller.srv import SetParameters, SetParametersRequest

def set_parameters(depth_speed, rotation_speed, linear_speed, delta):
    rospy.wait_for_service('set_parameters')
    try:
        set_params_service = rospy.ServiceProxy('set_parameters', SetParameters)
        request = SetParametersRequest(
            depth_speed=depth_speed,
            rotation_speed=rotation_speed,
            linear_speed=linear_speed,
            delta=delta
        )
        response = set_params_service(request)
        rospy.loginfo("Set parameters successful: %s", response.success)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('set_parameters_client')

    # Set parameters
    set_parameters(2, 2, 2, 0.02)
