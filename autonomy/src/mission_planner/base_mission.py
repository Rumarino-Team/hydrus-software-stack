#!/usr/bin/env python3
# mission_planner/base_mission.py
import math
from typing import Callable, Dict, List, Optional, Set

import actionlib
import rospy
from geometry_msgs.msg import Point, PoseStamped

from autonomy.msg import (
    Detection,
    Detections,
    NavigateToWaypointAction,
    NavigateToWaypointFeedback,
    NavigateToWaypointGoal,
)
from mission_planner.mission_tree import MissionTreeNode
from mission_planner.task_parameters import TaskParameters

# Change to absolute imports
from mission_planner.types import MissionObject, TaskType


class BaseMission:
    """
    Base class for all missions. Provides common functionality for mission execution.
    """

    def __init__(self, name="Base Mission"):
        # Mission metadata
        self.name = name
        self.completed = False
        self.active = False

        # Mission progress tracking
        self.completion_percentage = 0.0
        self.total_tasks = 0
        self.completed_tasks = 0
        self.current_task_progress = 0.0

        # Configuration
        self.cls_names = {}  # To be defined by subclass
        self.mission_tree_root: Optional[MissionTreeNode] = None

        # Mutable Data
        self.mission_objects: Set[MissionObject] = set()
        self.detected_objects: Set[str] = set()
        self.submarine_pose: Optional[PoseStamped] = None
        self.current_detections: Optional[Detections] = None

        # ROS Setup
        rospy.Subscriber("/detector/box_detection", Detections, self.detection_callback)
        rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, self.pose_callback)
        self.controller_client = actionlib.SimpleActionClient(
            "controller_action", NavigateToWaypointAction
        )

        try:
            rospy.loginfo(f"[{self.name}] Waiting for action server to start...")
            self.controller_client.wait_for_server(timeout=rospy.Duration(5.0))
            rospy.loginfo(f"[{self.name}] Action server started.")
        except rospy.ROSException:
            rospy.logwarn(f"[{self.name}] Action server not available after timeout.")

        self.calculate_distance = lambda pos1, pos2: math.sqrt(
            (pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2
        )

        # Task parameters
        self.task_params = TaskParameters()
        self.current_task_start_time = None
        self.current_waypoint_index = 0
        self.waypoints: List[Point] = []

        # For feedback
        self.current_task_name = "None"
        self.current_object_name = "None"
        self.current_phase = "Initialization"

    # ROS Callbacks
    def detection_callback(self, msg: Detections):
        """Callback for detection messages"""
        self.current_detections = msg

    def pose_callback(self, msg: PoseStamped):
        """Callback for pose messages"""
        self.submarine_pose = msg

    # Utility Functions
    def search_mission_object(self, object_name: str) -> Optional[MissionObject]:
        """Find a mission object by name"""
        for item in self.mission_objects:
            if item.object_name == object_name:
                return item
        return None

    def generate_circle_waypoints(
        self, center_point: Point, radius: float, num_points: int
    ) -> List[Point]:
        """Generate waypoints in a circle around a center point"""
        waypoints = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center_point.x + radius * math.cos(angle)
            y = center_point.y + radius * math.sin(angle)
            z = center_point.z
            waypoints.append(Point(x=x, y=y, z=z))
        return waypoints

    # Execution Methods for Specific Tasks
    def execute_move_to_center(self, object_position: Point) -> bool:
        """Move to the center of an object"""
        goal = NavigateToWaypointGoal()
        goal.target_point = object_position
        self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)
        self.controller_client.wait_for_result()
        result = self.controller_client.get_result()
        return result.success if result else False

    def execute_move_around(self, object_position: Point) -> bool:
        """Move around an object in a circle"""
        if not self.waypoints:
            self.waypoints = self.generate_circle_waypoints(
                object_position, self.task_params.radius, self.task_params.circle_points
            )
            self.current_waypoint_index = 0

        if self.current_waypoint_index < len(self.waypoints):
            goal = NavigateToWaypointGoal()
            goal.target_point = self.waypoints[self.current_waypoint_index]
            self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)
            self.controller_client.wait_for_result()

            # Update progress based on waypoints completed
            if self.controller_client.get_result().success:
                self.current_waypoint_index += 1
                if len(self.waypoints) > 0:
                    self.current_task_progress = (
                        self.current_waypoint_index / len(self.waypoints)
                    ) * 100

                if self.current_waypoint_index >= len(self.waypoints):
                    self.waypoints = []
                    self.current_waypoint_index = 0
                    self.current_task_progress = 100.0
                    return True
                return False
            return False
        return True

    def execute_hold_position(self, position: Point) -> bool:
        """Hold a position for a certain amount of time"""
        if self.current_task_start_time is None:
            self.current_task_start_time = rospy.Time.now()
            goal = NavigateToWaypointGoal()
            goal.target_point = position
            self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)

        # Update progress based on time elapsed
        elapsed_time = (rospy.Time.now() - self.current_task_start_time).to_sec()
        self.current_task_progress = min(
            100.0, (elapsed_time / self.task_params.hold_time) * 100
        )

        if elapsed_time >= self.task_params.hold_time:
            self.current_task_start_time = None
            self.current_task_progress = 100.0
            return True
        return False

    def feedback_callback(self, feedback: NavigateToWaypointFeedback):
        """Callback for action feedback"""
        rospy.loginfo(
            f"[{self.name} | Task: {self.current_task_name} | Object: {self.current_object_name}] "
            f"Distance to target: {feedback.distance_to_target}"
        )

    def update_detected_objects(self):
        """Update the list of detected objects"""
        if not self.current_detections:
            return

        for detection in self.current_detections.detections:
            if detection.cls not in self.cls_names:
                continue

            object_name = self.cls_names[detection.cls]
            if self.submarine_pose is None:
                continue

            new_distance = self.calculate_distance(
                detection.point, self.submarine_pose.pose.position
            )

            if object_name not in self.detected_objects:
                self.detected_objects.add(object_name)
                mission_obj = MissionObject(
                    position=detection.point,
                    object_name=object_name,
                    distance=new_distance,
                )
                self.mission_objects = {
                    obj
                    for obj in self.mission_objects
                    if obj.object_name != object_name
                }
                self.mission_objects.add(mission_obj)
            else:
                existing_obj = self.search_mission_object(object_name)
                if existing_obj:
                    checkpoint_margin = 1.0  # meters
                    checkpoints = [10.0, 5.0, 2.0]  # example checkpoints
                    update_required = False

                    if new_distance < existing_obj.distance - checkpoint_margin:
                        update_required = True
                    for cp in checkpoints:
                        if existing_obj.distance > cp and new_distance <= cp:
                            update_required = True
                            break

                    if update_required:
                        updated_obj = MissionObject(
                            position=detection.point,
                            object_name=object_name,
                            distance=new_distance,
                        )
                        self.mission_objects = {
                            obj
                            for obj in self.mission_objects
                            if obj.object_name != object_name
                        }
                        self.mission_objects.add(updated_obj)

    def create_action(
        self, task: TaskType, object_cls: Optional[str]
    ) -> Callable[[], bool]:
        """Create an action function for a task"""

        def action():
            self.current_task_name = task.name
            self.current_object_name = object_cls if object_cls is not None else "N/A"

            task_completed = False

            if task == TaskType.SURFACE:
                if self.submarine_pose is None:
                    rospy.logwarn("Submarine pose not available for SURFACE task.")
                    return False
                surface_point = Point(
                    x=self.submarine_pose.pose.position.x,
                    y=self.submarine_pose.pose.position.y,
                    z=0.0,
                )
                task_completed = self.execute_move_to_center(surface_point)
            else:
                if object_cls is None:
                    rospy.logwarn(f"Object class not specified for task {task.name}")
                    return False
                mission_object = self.search_mission_object(object_cls)
                if mission_object is None:
                    rospy.logwarn(f"No mission object found for {object_cls}")
                    return False

                if task == TaskType.MOVE_TO_CENTER:
                    task_completed = self.execute_move_to_center(
                        mission_object.position
                    )
                elif task == TaskType.MOVE_AROUND:
                    task_completed = self.execute_move_around(mission_object.position)
                elif task == TaskType.HOLD_POSITION:
                    task_completed = self.execute_hold_position(mission_object.position)
                else:
                    rospy.logwarn(f"Unsupported task: {task.name}")
                    return False

            if task_completed:
                self.completed_tasks += 1
                self.update_completion_percentage()

            return task_completed

        return action

    def execute_search(self) -> bool:
        """Execute a search pattern"""
        rospy.loginfo("Executing search pattern for object...")
        rospy.sleep(2)  # Simulate search delay
        return True

    def update_completion_percentage(self):
        """Update the mission completion percentage"""
        if self.total_tasks > 0:
            self.completion_percentage = (self.completed_tasks / self.total_tasks) * 100
        else:
            self.completion_percentage = 0.0

    def count_total_tasks(self):
        """Count the total number of tasks in the mission tree"""
        self.total_tasks = 0
        if self.mission_tree_root:
            self._count_tasks(self.mission_tree_root)

    def _count_tasks(self, node):
        """Recursive helper to count tasks"""
        if node is None:
            return

        if node.action is not None:
            self.total_tasks += 1

        for child in node.children:
            self._count_tasks(child)

    def build_mission_tree(self) -> MissionTreeNode:
        """Build the mission tree - to be implemented by subclass"""
        raise NotImplementedError("Subclass must implement build_mission_tree()")

    def get_status(self) -> dict:
        """Get the mission status"""
        return {
            "name": self.name,
            "completed": self.completed,
            "active": self.active,
            "completion_percentage": self.completion_percentage,
            "current_phase": self.current_phase,
            "current_task": self.current_task_name,
            "current_object": self.current_object_name,
            "task_progress": self.current_task_progress,
            "detected_objects": list(self.detected_objects),
        }

    def run(self):
        """Run one iteration of the mission - to be implemented by subclass"""
        raise NotImplementedError("Subclass must implement run()")

    def activate(self):
        """Activate the mission"""
        self.active = True
        rospy.loginfo(f"Mission '{self.name}' activated")

    def deactivate(self):
        """Deactivate the mission"""
        self.active = False
        rospy.loginfo(f"Mission '{self.name}' deactivated")

    def reset(self):
        """Reset the mission state"""
        self.completed = False
        self.completion_percentage = 0.0
        self.completed_tasks = 0
        self.current_task_progress = 0.0
        self.current_phase = "Initialization"
        if self.mission_tree_root:
            self.mission_tree_root.reset()
        rospy.loginfo(f"Mission '{self.name}' reset")
