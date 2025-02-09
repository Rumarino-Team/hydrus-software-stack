# mission_planner/prequalification_mission.py
import math
from typing import List, Optional, Set, Callable

import rospy
import actionlib
from geometry_msgs.msg import Point, PoseStamped
from autonomy.msg import Detection, Detections  
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointGoal, NavigateToWaypointFeedback, NavigateToWaypointResult

from mission_planner.types import TaskType, MissionObject
from mission_planner.mission_tree import MissionTreeNode
from mission_planner.task_parameters import TaskParameters

class PreQualificationMission:
    def __init__(self):
        # Configuration
        self.cls_names = {1: 'Gate', 2: 'Buoy'}
        self.mission_tree_root: Optional[MissionTreeNode] = None

        # Mutable Data
        self.mission_objects: Set[MissionObject] = set()
        self.detected_objects: Set[str] = set()
        self.submarine_pose: Optional[PoseStamped] = None
        self.current_detections: Optional[Detections] = None

        # ROS Setup
        rospy.Subscriber("/detector/box_detection", Detections, self.detection_callback)
        rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, self.pose_callback)
        self.controller_client = actionlib.SimpleActionClient('controller_action', NavigateToWaypointAction)
        
        rospy.loginfo("Waiting for action server to start...")
        self.controller_client.wait_for_server()
        rospy.loginfo("Action server started.")

        self.calculate_distance = lambda pos1, pos2: math.sqrt((pos1.x - pos2.x)**2 +
                                                               (pos1.y - pos2.y)**2 +
                                                               (pos1.z - pos2.z)**2)

        # Task parameters
        self.task_params = TaskParameters()
        self.current_task_start_time = None
        self.current_waypoint_index = 0
        self.waypoints: List[Point] = []

        # For feedback
        self.current_task_name = "None"
        self.current_object_name = "None"

        # Build the mission tree
        self.mission_tree_root = self.build_mission_tree()

    # ROS Callbacks
    def detection_callback(self, msg: Detections):
        self.current_detections = msg

    def pose_callback(self, msg: PoseStamped):
        self.submarine_pose = msg

    # Utility Functions
    def search_mission_object(self, object_name: str) -> Optional[MissionObject]:
        for item in self.mission_objects:
            if item.object_name == object_name:
                return item
        return None

    def generate_circle_waypoints(self, center_point: Point, radius: float, num_points: int) -> List[Point]:
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
        goal = NavigateToWaypointGoal()
        goal.target_point = object_position
        self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)
        self.controller_client.wait_for_result()
        result = self.controller_client.get_result()
        return result.success if result else False

    def execute_move_around(self, object_position: Point) -> bool:
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
                if self.current_waypoint_index >= len(self.waypoints):
                    self.waypoints = []
                    self.current_waypoint_index = 0
                    return True
                return False
            return False
        return True

    def execute_hold_position(self, position: Point) -> bool:
        if self.current_task_start_time is None:
            self.current_task_start_time = rospy.Time.now()
            goal = NavigateToWaypointGoal()
            goal.target_point = position
            self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)

        if (rospy.Time.now() - self.current_task_start_time).to_sec() >= self.task_params.hold_time:
            self.current_task_start_time = None
            return True
        return False

    def feedback_callback(self, feedback: NavigateToWaypointFeedback):
        rospy.loginfo(f"[Task: {self.current_task_name} | Object: {self.current_object_name}] Distance to target: {feedback.distance_to_target}")

    def update_detected_objects(self):
        if not self.current_detections:
            return

        for detection in self.current_detections.detections:
            if detection.cls not in self.cls_names:
                rospy.logwarn(f"Unknown object class ID: {detection.cls}")
                continue

            object_name = self.cls_names[detection.cls]
            if self.submarine_pose is None:
                continue

            new_distance = self.calculate_distance(detection.point, self.submarine_pose.pose.position)

            if object_name not in self.detected_objects:
                self.detected_objects.add(object_name)
                rospy.loginfo(f"New object detected: {object_name}")
                mission_obj = MissionObject(position=detection.point, object_name=object_name, distance=new_distance)
                self.mission_objects = {obj for obj in self.mission_objects if obj.object_name != object_name}
                self.mission_objects.add(mission_obj)
                rospy.loginfo(f"Added mission object: {object_name} at distance {new_distance:.2f}")
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
                        rospy.loginfo(f"Updating position for {object_name}: old distance {existing_obj.distance:.2f} -> new distance {new_distance:.2f}")
                        updated_obj = MissionObject(position=detection.point, object_name=object_name, distance=new_distance)
                        self.mission_objects = {obj for obj in self.mission_objects if obj.object_name != object_name}
                        self.mission_objects.add(updated_obj)

    def create_action(self, task: TaskType, object_cls: Optional[str]) -> Callable[[], bool]:
        def action():
            self.current_task_name = task.name
            self.current_object_name = object_cls if object_cls is not None else "N/A"

            if task == TaskType.SURFACE:
                if self.submarine_pose is None:
                    rospy.logwarn("Submarine pose not available for SURFACE task.")
                    return False
                surface_point = Point(
                    x=self.submarine_pose.pose.position.x,
                    y=self.submarine_pose.pose.position.y,
                    z=0.0
                )
                return self.execute_move_to_center(surface_point)
            else:
                if object_cls is None:
                    rospy.logwarn(f"Object class not specified for task {task.name}")
                    return False
                mission_object = self.search_mission_object(object_cls)
                if mission_object is None:
                    rospy.logwarn(f"No mission object found for {object_cls}")
                    return False
                if task == TaskType.MOVE_TO_CENTER:
                    return self.execute_move_to_center(mission_object.position)
                elif task == TaskType.MOVE_AROUND:
                    return self.execute_move_around(mission_object.position)
                elif task == TaskType.HOLD_POSITION:
                    return self.execute_hold_position(mission_object.position)
                else:
                    rospy.logwarn(f"Unsupported task: {task.name}")
                    return False
        return action

    def execute_search(self) -> bool:
        rospy.loginfo("Executing search pattern for object...")
        rospy.sleep(2)  # Simulate search delay
        return True

    def build_mission_tree(self) -> MissionTreeNode:
        root = MissionTreeNode(
            task=TaskType.HOLD_POSITION,
            name="Mission Start",
            action=lambda: True  # Grouping node
        )

        # Gate Branch
        search_gate_node = MissionTreeNode(
            task=TaskType.SEARCH,
            object_cls="Gate",
            name="Search for Gate",
            condition=lambda: "Gate" not in self.detected_objects,
            action=self.execute_search
        )
        approach_gate_node = MissionTreeNode(
            task=TaskType.MOVE_TO_CENTER,
            object_cls="Gate",
            name="Approach Gate",
            condition=lambda: "Gate" in self.detected_objects,
            action=self.create_action(TaskType.MOVE_TO_CENTER, "Gate")
        )
        gate_branch_node = MissionTreeNode(
            task=TaskType.HOLD_POSITION,
            name="Gate Branch",
            action=lambda: True  # Grouping node
        )
        gate_branch_node.add_child(search_gate_node)
        gate_branch_node.add_child(approach_gate_node)
        root.add_child(gate_branch_node)

        # Buoy Branch
        buoy_node = MissionTreeNode(
            task=TaskType.MOVE_AROUND,
            object_cls="Buoy",
            name="Circle Buoy",
            condition=lambda: "Buoy" in self.detected_objects,
            action=self.create_action(TaskType.MOVE_AROUND, "Buoy")
        )
        root.add_child(buoy_node)

        # Re-approach Gate
        reapproach_gate_node = MissionTreeNode(
            task=TaskType.MOVE_TO_CENTER,
            object_cls="Gate",
            name="Re-approach Gate",
            condition=lambda: "Gate" in self.detected_objects,
            action=self.create_action(TaskType.MOVE_TO_CENTER, "Gate")
        )
        root.add_child(reapproach_gate_node)

        # Surface
        surface_node = MissionTreeNode(
            task=TaskType.SURFACE,
            name="Surface",
            condition=lambda: True,
            action=self.create_action(TaskType.SURFACE, None)
        )
        root.add_child(surface_node)

        return root

    def run(self):
        if not self.submarine_pose:
            rospy.logwarn_throttle(1, "Waiting for submarine pose...")
            return

        self.update_detected_objects()

        if self.mission_tree_root and not self.mission_tree_root.completed:
            rospy.loginfo("Executing mission tree...")
            self.mission_tree_root.execute()
        else:
            rospy.loginfo("Mission tree completed or not available.")
