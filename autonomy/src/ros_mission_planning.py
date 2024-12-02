import math
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Callable, Set

# ROS Dependencies
import rospy
import actionlib
from geometry_msgs.msg import Point, PoseStamped
from autonomy.msg import Detection, Detections  
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointGoal, NavigateToWaypointFeedback, NavigateToWaypointResult

@dataclass
class MissionObject:
    position: Point
    object_name: str
    distance: float

class TaskType(Enum):
    MOVETOCENTER = 1
    MOVEAROUND = 2

class Status(Enum):
    COMPLETED = 1
    ONPROGRESS = 2
    NONCALLED = 3

@dataclass
class MissionInstructions:
    task: TaskType
    object_cls: str
    conditions: Callable
    status: Status = Status.NONCALLED

class PreQualificationMission:
    def __init__(self):

        # //////////////////////////////
        # /////// CONFIGURATION ////////
        # //////////////////////////////
        self.cls_names = {1: 'Gate', 2: 'Buoy'}
        self.instructions = deque()
        # Define mission instructions
        self.instructions.extend([
            MissionInstructions(task=TaskType.MOVETOCENTER, object_cls="Gate", conditions=lambda: True),
            MissionInstructions(task=TaskType.MOVEAROUND, object_cls="Buoy", conditions=lambda: True),
            MissionInstructions(task=TaskType.MOVETOCENTER, object_cls="Gate", conditions=lambda: True)
        ])

        # //////////////////////////////
        # //////// MUTABLE DATA ///////
        # //////////////////////////////
        self.current_instruction: Optional[MissionInstructions] = None
        self.mission_objects: Set[MissionObject] = set()
        self.detected_objects: Set[str] = set()
        self.submarine_pose: Optional[PoseStamped] = None
        self.current_detections: Optional[Detections] = None

        # ///////////////////////////////
        # ////// INIT ROS DATA /////////
        # ///////////////////////////////
        rospy.Subscriber("/detector/box_detection", Detections, self.detection_callback)
        rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, self.pose_callback)
        self.controller_client = actionlib.SimpleActionClient('controller_action', NavigateToWaypointAction)
        
        rospy.loginfo("Waiting for action server to start...")
        self.controller_client.wait_for_server()
        rospy.loginfo("Action server started.")

        self.calculate_distance = lambda pos1, pos2: math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)

    def detection_callback(self, msg):
        self.current_detections = msg

    def pose_callback(self, msg):
        self.submarine_pose = msg

    def search_mission_object(self, object_name: str):
        for item in self.mission_objects:
            if item.object_name == object_name:
                return item
        return None

    def execute_task(self, task: MissionInstructions):
        if task.object_cls in self.detected_objects:
            mission_object = self.search_mission_object(task.object_cls)

            # Send goal to action server
            goal = NavigateToWaypointGoal(target_position=mission_object.position)
            self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)
            
            # Wait for the result
            self.client.wait_for_result()
            result = self.controller_client.get_result()

            if result.success:
                rospy.loginfo(f"Successfully completed task: {task.task} for object: {task.object_cls}")
                return True
            else:
                rospy.logwarn(f"Failed to complete task: {task.task} for object: {task.object_cls}")
                return False

    def feedback_callback(self, feedback: NavigateToWaypointFeedback):
        rospy.loginfo(f"Distance to target: {feedback.distance_to_target}")

    def run(self):
        if not self.current_detections:
            return
        
        for detection in self.current_detections.detections:
            if self.cls_names[detection.cls] == "Gate" and "Gate" not in self.detected_objects:
                self.detected_objects.add("Gate")
                distance = self.calculate_distance(detection.point, self.submarine_pose.pose.position)
                self.mission_objects.add(MissionObject(detection.point, "Gate", distance=distance)) 
            elif self.cls_names[detection.cls] == "Buoy" and "Buoy" not in self.detected_objects:
                self.detected_objects.add("Buoy")
                distance = self.calculate_distance(detection.point, self.submarine_pose.pose.position)
                self.mission_objects.add(MissionObject(detection.point, "Buoy", distance=distance))

        if not self.current_instruction and self.instructions:
            self.current_instruction = self.instructions.popleft()

        if self.current_instruction:
            if self.current_instruction.status == Status.NONCALLED:
                self.current_instruction.status = Status.ONPROGRESS
                rospy.loginfo(f"Starting task: {self.current_instruction.task} for object: {self.current_instruction.object_cls}")

            if self.current_instruction.status == Status.ONPROGRESS:
                success = self.execute_task(self.current_instruction)
                if success:
                    self.current_instruction.status = Status.COMPLETED
                    rospy.loginfo(f"Completed task: {self.current_instruction.task} for object: {self.current_instruction.object_cls}")
                    self.current_instruction = None

def main():
    rospy.init_node('prequalification_mission_node')
    mission = PreQualificationMission()
    rate = rospy.Rate(1) 
    while not rospy.is_shutdown():
        mission.run()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
