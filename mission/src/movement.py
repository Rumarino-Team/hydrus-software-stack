import rospy
import time
import smach
import random
from geometry_msgs.msg import Point, PoseStamped
import math
from controller.srv import NavigateToWaypoint, NavigateToWaypointRequest

class UpdatePoseState(smach.State):
    """Parameters:

    edge_case_callback - callback function for edge case detection
    point: None
    
    """
    def __init__(self, edge_case_callback, next_state_callback=None, point=None, threshold=1.2, stabilization_time=1,
                 outcomes=['success', 'edge_case_detected', 'aborted'], input_keys=['shared_data'], output_keys=['shared_data']):
        smach.State.__init__(self, outcomes=outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)
        self.edge_case_callback = edge_case_callback
        self.next_state_callback = next_state_callback
        self.point = point
        self.threshold = threshold
        self.stabilization_time = stabilization_time
        self.timeout_duration = 30

    @staticmethod
    def pose_reached( pose_obj, destination_point, threshold):
        # Check if the current pose is within a certain threshold of the destination pose
        # The function 'compare_poses' should return True if the poses are similar within the threshold
        # print("current_pose",current_pose)

        current_pose = pose_obj.pose
        if not isinstance(pose_obj, PoseStamped):
            rospy.logerr("current_pose must be an instance of Pose and got of type: " + str(type(current_pose)))

        position_diff = math.sqrt(
                (current_pose.position.x - destination_point.x) ** 2 +
                (current_pose.position.y - destination_point.y) ** 2 +
                (current_pose.position.z - destination_point.z) ** 2
            )

        return position_diff <= threshold 

    def call_movement(self, target_point):
        rospy.wait_for_service('navigate_to_waypoint')
        try:
            navigate_service = rospy.ServiceProxy('navigate_to_waypoint', NavigateToWaypoint)
            request = NavigateToWaypointRequest(target_point=target_point)
            response = navigate_service(request)
            rospy.loginfo("Service call successful: %s", response.success)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return 'aborted'

    def loop_monitor(self, userdata, target_point):
            shared_data = userdata.shared_data
            start_time = rospy.Time.now().secs  # Start time for timeout calculation
            # For the target poses we only change the yaw orientation into the quaternion
            while not rospy.is_shutdown() and rospy.Time.now().secs - start_time < self.timeout_duration:
                if self.pose_reached(shared_data.zed_data["pose"], target_point, self.threshold):
                    rospy.loginfo("Destination reached. Verifying stabilization.")
                    # Ensure stabilization for the configured time
                    stabilization_start = rospy.Time.now()
                    while rospy.Time.now() - stabilization_start < rospy.Duration(0.5):
                        if not self.pose_reached(shared_data.zed_data["pose"], target_point, self.threshold):
                            break
                        rospy.sleep(0.1)
                    else:  # If the loop completes without breaking
                        rospy.loginfo("Destination stabilized.")
                        return 'success'
            rospy.sleep(0.1)
            print("Monitoring loop")
            
    def execute(self, userdata):
        self.call_movement(self.point)
        return self.loop_monitor(userdata, self.point)

class UpdatePoseToObjectState(UpdatePoseState):
    def __init__(self, desired_object_name, edge_case_callback=None, next_state_callback=None, point=None):
        super(UpdatePoseToObjectState, self).__init__(outcomes=['success', 'edge_case_detected', 'aborted', 'object_not_detected'],
                                                      input_keys=['shared_data'],
                                                      output_keys=['shared_data', 'detected_object'],
                                                      edge_case_callback=edge_case_callback,
                                                      next_state_callback=next_state_callback,
                                                      point=point)
        self.desired_object_name = desired_object_name

    def execute(self, userdata):
        shared_data = userdata.shared_data
        detections = shared_data.detector["box_detection"]
        if rospy.is_shutdown():
            return "aborted"
        if detections == None:
            rospy.logwarn("Box_detection not yet available")
            time.sleep(1)
            return "object_not_detected"
        for detection in  detections.detections:
            print(f"detection = {detection}, cls = {detection.cls}")
            if self.desired_object_name in detections.class_names:
                userdata.detected_object = detection
                if self.point:  # Update the Pose with the Offset of a Point
                    target_point = Point(detection.point.x + self.point.x, detection.point.y + self.point.y, detection.point.z + self.point.z)
                    self.call_movement(target_point)
                    return self.loop_monitor(userdata, target_point)
                else:
                    self.call_movement(detection.point)
                    return self.loop_monitor(userdata, detection.point)

        else:
            return "object_not_detected"

class ContinuePoseObjectMovement(UpdatePoseState):
    def __init__(self, offset_point, edge_case_callback=None,next_state_callback=None ):
        super(ContinuePoseObjectMovement, self).__init__(outcomes=['success', 'edge_case_detected', 'aborted'],
                                                      input_keys=['shared_data', "detected_object"],
                                                      output_keys=['edge_case'],
                                                 edge_case_callback=edge_case_callback,
                                                   next_state_callback=next_state_callback)
        self.offset_point = offset_point

    def execute(self, userdata):
        shared_data = userdata.shared_data
        detected_object = userdata.detected_object
        target_point = Point(detected_object.point.x + self.offset_point.x,
                             detected_object.point.y + self.offset_point.y,
                             detected_object.point.z + self.offset_point.z)
        self.call_movement(target_point)
        return self.loop_monitor(userdata, target_point)
