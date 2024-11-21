import rospy
from autonomy.srv import SetColorFilterConfig  

def set_color_filter_config_client(tolerance, min_confidence, min_area, rgb_range):
    rospy.wait_for_service('set_color_filter_config')
    try:
        set_config = rospy.ServiceProxy('set_color_filter_config', SetColorFilterConfig)
        response = set_config(tolerance, min_confidence, min_area, rgb_range)
        print(response.message)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('set_color_filter_config_client')
    set_color_filter_config_client(0.5, 0.25, 0.15, [0, 255, 0])
