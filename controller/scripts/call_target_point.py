#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from controller.srv import NavigateToWaypoint, NavigateToWaypointRequest

def call_navigate_service(target_point):
    rospy.wait_for_service('navigate_to_waypoint')
    try:
        navigate_service = rospy.ServiceProxy('/navigate_to_waypoint', NavigateToWaypoint)
        request = NavigateToWaypointRequest(target_point=target_point)
        response = navigate_service(request)
        rospy.loginfo("Service call successful: %s", response.success)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('navigate_to_waypoint_client')
    
    # Define a target point
    target_point = Point(x=1.0, y=2.0, z=3.0)

    # Call the service
    call_navigate_service(target_point)
