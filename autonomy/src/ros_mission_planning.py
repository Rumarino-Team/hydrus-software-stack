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


# Define TaskType (ensure names are consistent)
class TaskType(Enum):
    MOVE_TO_CENTER = 1    # Move directly to the center of an object
    MOVE_AROUND = 2       # Circle around an object
    MOVE_THROUGH = 3      # Move through an object (e.g., a gate)
    HOLD_POSITION = 4     # Maintain current position
    SURFACE = 5           # Move to the surface
    SEARCH = 6            # A search pattern (new type for branch)


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

@dataclass
class TaskParameters:
    # Parameters that can be configured for different tasks
    radius: float = 2.0           # For MOVE_AROUND - radius of circular motion
    circle_points: int = 8        # For MOVE_AROUND - number of points in circle
    approach_distance: float = 1.0 # Distance to maintain from target
    hold_time: float = 5.0        # Time to hold position in seconds

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

        # Add task parameters
        self.task_params = TaskParameters()
        
        # Add task completion tracking
        self.current_task_start_time = None
        self.current_waypoint_index = 0
        self.waypoints = []

    def detection_callback(self, msg):
        self.current_detections = msg

    def pose_callback(self, msg):
        self.submarine_pose = msg

    def search_mission_object(self, object_name: str):
        for item in self.mission_objects:
            if item.object_name == object_name:
                return item
        return None

    def generate_circle_waypoints(self, center_point: Point, radius: float, num_points: int) -> List[Point]:
        """Generate waypoints in a circle around a center point"""
        waypoints = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center_point.x + radius * math.cos(angle)
            y = center_point.y + radius * math.sin(angle)
            z = center_point.z
            waypoints.append(Point(x=x, y=y, z=z))
        return waypoints

    def execute_move_to_center(self, object_position: Point) -> bool:
        """Execute movement directly to the center of an object"""
        goal = NavigateToWaypointGoal()
        goal.target_point = object_position
        self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)
        self.controller_client.wait_for_result()
        return self.controller_client.get_result().success

    def execute_move_around(self, object_position: Point) -> bool:
        """Execute circular movement around an object"""
        if not self.waypoints:
            self.waypoints = self.generate_circle_waypoints(
                object_position, 
                self.task_params.radius,
                self.task_params.circle_points
            )
            self.current_waypoint_index = 0

        if self.current_waypoint_index < len(self.waypoints):
            goal = NavigateToWaypointGoal()
            goal.target_point = self.waypoints[self.current_waypoint_index]
            self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)
            self.controller_client.wait_for_result()
            
            if self.controller_client.get_result().success:
                self.current_waypoint_index += 1
                return self.current_waypoint_index >= len(self.waypoints)
            return False
        return True

    def execute_hold_position(self, position: Point) -> bool:
        """Hold position for a specified duration"""
        if self.current_task_start_time is None:
            self.current_task_start_time = rospy.Time.now()
            goal = NavigateToWaypointGoal()
            goal.target_point = position
            self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)

        if (rospy.Time.now() - self.current_task_start_time).to_sec() >= self.task_params.hold_time:
            self.current_task_start_time = None
            return True
        return False

    def execute_task(self, task: MissionInstructions) -> bool:
        """Enhanced task execution with different behaviors"""
        if task.object_cls not in self.detected_objects:
            rospy.logwarn(f"Object {task.object_cls} not detected yet")
            return False

        mission_object = self.search_mission_object(task.object_cls)
        if not mission_object:
            return False

        success = False
        if task.task == TaskType.MOVE_TO_CENTER:
            success = self.execute_move_to_center(mission_object.position)
        
        elif task.task == TaskType.MOVE_AROUND:
            success = self.execute_move_around(mission_object.position)
        
        elif task.task == TaskType.HOLD_POSITION:
            success = self.execute_hold_position(mission_object.position)
        
        elif task.task == TaskType.SURFACE:
            surface_point = Point(
                x=self.submarine_pose.pose.position.x,
                y=self.submarine_pose.pose.position.y,
                z=0.0  # Assuming 0 is surface level
            )
            success = self.execute_move_to_center(surface_point)

        if success:
            rospy.loginfo(f"Successfully completed task: {task.task} for object: {task.object_cls}")
            # Reset task-specific variables
            self.waypoints = []
            self.current_waypoint_index = 0
            self.current_task_start_time = None
        
        return success

    def feedback_callback(self, feedback: NavigateToWaypointFeedback):
        rospy.loginfo(f"Distance to target: {feedback.distance_to_target}")

    def update_detected_objects(self):
        """Update the detected objects and mission objects based on current detections"""
        if not self.current_detections:
            return

        for detection in self.current_detections.detections:
            # Get object class name from detection
            if detection.cls not in self.cls_names:
                rospy.logwarn(f"Unknown object class ID: {detection.cls}")
                continue
            
            object_name = self.cls_names[detection.cls]
            
            # Add to detected objects set if not already present
            if object_name not in self.detected_objects:
                self.detected_objects.add(object_name)
                rospy.loginfo(f"New object detected: {object_name}")
                
                # Calculate distance from submarine to object
                if self.submarine_pose:
                    distance = self.calculate_distance(
                        detection.point,
                        self.submarine_pose.pose.position
                    )
                    
                    # Create new mission object
                    mission_obj = MissionObject(
                        position=detection.point,
                        object_name=object_name,
                        distance=distance
                    )
                    
                    # Update or add to mission objects
                    # Remove old instance if exists
                    self.mission_objects = {obj for obj in self.mission_objects 
                                         if obj.object_name != object_name}
                    self.mission_objects.add(mission_obj)
                    rospy.loginfo(f"Added mission object: {object_name} at distance {distance:.2f}")
            else:
                # Update position and distance for existing objects
                if self.submarine_pose:
                    distance = self.calculate_distance(
                        detection.point,
                        self.submarine_pose.pose.position
                    )
                    
                    # Update mission object with new position and distance
                    updated_obj = MissionObject(
                        position=detection.point,
                        object_name=object_name,
                        distance=distance
                    )
                    
                    # Remove old instance and add updated one
                    self.mission_objects = {obj for obj in self.mission_objects 
                                         if obj.object_name != object_name}
                    self.mission_objects.add(updated_obj)

    def run(self):
        """Main mission execution loop"""
        if not self.submarine_pose:
            rospy.logwarn_throttle(1, "Waiting for submarine pose...")
            return

        # Update detected objects from current detections
        self.update_detected_objects()

        # Execute current instruction if available
        if self.current_instruction:
            if self.current_instruction.status == Status.NONCALLED:
                self.current_instruction.status = Status.ONPROGRESS
                rospy.loginfo(f"Starting task: {self.current_instruction.task} for object: {self.current_instruction.object_cls}")

            if self.current_instruction.status == Status.ONPROGRESS:
                if self.execute_task(self.current_instruction):
                    self.current_instruction.status = Status.COMPLETED
                    self.current_instruction = None
        
        # Get next instruction if available
        elif self.instructions:
            self.current_instruction = self.instructions.popleft()

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
