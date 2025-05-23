#!/usr/bin/env python3
# mission_planner/slalom_mission.py
import math
from typing import List, Optional, Set, Callable, Dict, Tuple

import rospy
import actionlib
from geometry_msgs.msg import Point, PoseStamped
from autonomy.msg import Detection, Detections
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointGoal, NavigateToWaypointFeedback, NavigateToWaypointResult
from std_msgs.msg import String

# Change to absolute imports
from mission_planner.types import TaskType, MissionObject
from mission_planner.mission_tree import MissionTreeNode
from mission_planner.task_parameters import TaskParameters
from mission_planner.base_mission import BaseMission

class SlalomMission(BaseMission):
    """
    Mission for navigating a slalom course with red and white PVC pipes.
    The submarine must navigate through three sets of gates, keeping the red pipe
    on the same side throughout the course.
    """
    
    def __init__(self):
        # Initialize the base mission
        super().__init__(name="Slalom Navigation")
        
        # Configuration specific to Slalom
        self.cls_names = {1: 'WhitePipe', 2: 'RedPipe'}
        
        # Slalom specific parameters
        self.passing_side = None  # 'left' or 'right' of the red pipe
        self.gates_passed = 0
        self.total_gates = 3
        self.current_gate = 0
        self.gate_positions: List[Dict[str, Point]] = []
        self.gates_completed = []
        
        # Additional subscribers/publishers
        self.status_pub = rospy.Publisher('/mission_status', String, queue_size=10)
        
        # Build the mission tree
        self.mission_tree_root = self.build_mission_tree()
        
    def detect_gate(self, gate_number: int) -> bool:
        """Detect if both pipes of a gate are visible"""
        if not self.current_detections:
            return False
            
        # Count white and red pipes seen
        white_pipes = []
        red_pipes = []
        
        for detection in self.current_detections.detections:
            if detection.cls == 1:  # White pipe
                white_pipes.append(detection)
            elif detection.cls == 2:  # Red pipe
                red_pipes.append(detection)
        
        # For a gate to be detected, we need at least one white pipe and one red pipe
        if white_pipes and red_pipes:
            # If this is a new gate (not already in our gate_positions)
            if gate_number >= len(self.gate_positions):
                # Record the gate position
                self.gate_positions.append({
                    'white': white_pipes[0].point,  # Using the first detected white pipe
                    'red': red_pipes[0].point       # Using the first detected red pipe
                })
                rospy.loginfo(f"Gate {gate_number + 1} detected")
            return True
        return False
        
    def determine_passing_side(self) -> str:
        """Determine which side the submarine should pass on"""
        if self.passing_side is not None:
            return self.passing_side
            
        if not self.gate_positions or not self.submarine_pose:
            return "unknown"
            
        # Calculate if submarine is to the left or right of the red pipe
        sub_pos = self.submarine_pose.pose.position
        red_pipe = self.gate_positions[0]['red']
        
        # Simple 2D calculation - if submarine's Y coordinate is less than red pipe's Y,
        # then submarine is on the left side (in ROS coordinate system)
        if sub_pos.y < red_pipe.y:
            self.passing_side = "left"
        else:
            self.passing_side = "right"
            
        rospy.loginfo(f"Determined passing side: {self.passing_side} of red pipe")
        return self.passing_side
        
    def navigate_gate(self, gate_number: int) -> bool:
        """Navigate through a specific gate"""
        if gate_number >= len(self.gate_positions):
            rospy.logwarn(f"Gate {gate_number + 1} position not recorded yet")
            return False
            
        # Get the gate position
        gate = self.gate_positions[gate_number]
        red_pipe = gate['red']
        white_pipe = gate['white']
        
        # Calculate the midpoint to pass through
        # Adjust based on passing side
        passing_side = self.determine_passing_side()
        
        # Calculate a point to navigate to
        target_point = Point()
        
        if passing_side == "left":
            # Pass with red pipe on right, white pipe on left
            target_point.x = (white_pipe.x + red_pipe.x) / 2
            target_point.y = (white_pipe.y + red_pipe.y) / 2
            # Adjust to be more on the left side
            target_point.y -= 0.5  # Shift left by 0.5m
        else:
            # Pass with red pipe on left, white pipe on right
            target_point.x = (white_pipe.x + red_pipe.x) / 2
            target_point.y = (white_pipe.y + red_pipe.y) / 2
            # Adjust to be more on the right side
            target_point.y += 0.5  # Shift right by 0.5m
            
        target_point.z = (white_pipe.z + red_pipe.z) / 2  # Keep at the middle height
        
        # Navigate to this point
        goal = NavigateToWaypointGoal()
        goal.target_point = target_point
        self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)
        self.controller_client.wait_for_result()
        result = self.controller_client.get_result()
        
        # If navigation was successful, mark this gate as passed
        if result and result.success:
            if gate_number not in self.gates_completed:
                self.gates_completed.append(gate_number)
                self.gates_passed += 1
                rospy.loginfo(f"Successfully passed gate {gate_number + 1}")
            return True
        return False

    def execute_search_for_gate(self) -> bool:
        """Search for the next gate in the slalom course"""
        rospy.loginfo(f"Searching for gate {self.current_gate + 1}...")
        
        # Simple search pattern - rotate in place
        if self.submarine_pose is None:
            rospy.logwarn("Submarine pose not available for search.")
            return False
            
        # Create a point to rotate around
        center = self.submarine_pose.pose.position
        
        # Generate rotation waypoints
        if not self.waypoints:
            radius = 0.0  # Rotate in place
            self.waypoints = self.generate_circle_waypoints(center, radius, 8)
            self.current_waypoint_index = 0
            
        # Move to next waypoint in rotation
        if self.current_waypoint_index < len(self.waypoints):
            goal = NavigateToWaypointGoal()
            goal.target_point = self.waypoints[self.current_waypoint_index]
            self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)
            self.controller_client.wait_for_result()
            
            # Check if we've detected the gate during this rotation
            if self.detect_gate(self.current_gate):
                self.waypoints = []
                self.current_waypoint_index = 0
                return True
                
            # Move to next rotation point
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                # Full rotation completed, reset and continue searching
                self.waypoints = []
                self.current_waypoint_index = 0
                
        return False

    def build_mission_tree(self) -> MissionTreeNode:
        """Build the mission tree for the slalom mission"""
        # Main mission node
        root = MissionTreeNode(
            task=TaskType.HOLD_POSITION,
            name="Slalom Mission Start",
            action=lambda: True  # Grouping node
        )
        
        # Configure task parameters for the mission
        self.task_params.search_timeout = 30.0  # seconds
        self.task_params.hold_time = 3.0  # seconds
        
        # First Gate
        gate1_branch = MissionTreeNode(
            task=TaskType.HOLD_POSITION,
            name="Gate 1",
            action=lambda: True  # Grouping node
        )
        
        search_gate1 = MissionTreeNode(
            task=TaskType.SEARCH,
            name="Search for Gate 1",
            condition=lambda: self.current_gate == 0 and not self.detect_gate(0),
            action=lambda: self.execute_search_for_gate()
        )
        
        navigate_gate1 = MissionTreeNode(
            task=TaskType.MOVE_TO_CENTER,
            name="Navigate Gate 1",
            condition=lambda: self.current_gate == 0 and self.detect_gate(0),
            action=lambda: self.navigate_gate(0) and self.increment_gate()
        )
        
        gate1_branch.add_child(search_gate1)
        gate1_branch.add_child(navigate_gate1)
        root.add_child(gate1_branch)
        
        # Second Gate
        gate2_branch = MissionTreeNode(
            task=TaskType.HOLD_POSITION,
            name="Gate 2",
            action=lambda: True  # Grouping node
        )
        
        search_gate2 = MissionTreeNode(
            task=TaskType.SEARCH,
            name="Search for Gate 2",
            condition=lambda: self.current_gate == 1 and not self.detect_gate(1),
            action=lambda: self.execute_search_for_gate()
        )
        
        navigate_gate2 = MissionTreeNode(
            task=TaskType.MOVE_TO_CENTER,
            name="Navigate Gate 2",
            condition=lambda: self.current_gate == 1 and self.detect_gate(1),
            action=lambda: self.navigate_gate(1) and self.increment_gate()
        )
        
        gate2_branch.add_child(search_gate2)
        gate2_branch.add_child(navigate_gate2)
        root.add_child(gate2_branch)
        
        # Third Gate
        gate3_branch = MissionTreeNode(
            task=TaskType.HOLD_POSITION,
            name="Gate 3",
            action=lambda: True  # Grouping node
        )
        
        search_gate3 = MissionTreeNode(
            task=TaskType.SEARCH,
            name="Search for Gate 3",
            condition=lambda: self.current_gate == 2 and not self.detect_gate(2),
            action=lambda: self.execute_search_for_gate()
        )
        
        navigate_gate3 = MissionTreeNode(
            task=TaskType.MOVE_TO_CENTER,
            name="Navigate Gate 3",
            condition=lambda: self.current_gate == 2 and self.detect_gate(2),
            action=lambda: self.navigate_gate(2) and self.increment_gate()
        )
        
        gate3_branch.add_child(search_gate3)
        gate3_branch.add_child(navigate_gate3)
        root.add_child(gate3_branch)
        
        # Surface at the end
        surface_node = MissionTreeNode(
            task=TaskType.SURFACE,
            name="Surface",
            condition=lambda: self.current_gate >= 3,
            action=self.create_action(TaskType.SURFACE, None)
        )
        root.add_child(surface_node)
        
        return root
    
    def increment_gate(self) -> bool:
        """Increment the current gate counter and return true"""
        self.current_gate += 1
        return True
    
    def get_status(self) -> dict:
        """Return the current status of the mission"""
        return {
            "name": self.name,
            "gates_passed": self.gates_passed,
            "total_gates": self.total_gates,
            "current_gate": self.current_gate,
            "passing_side": self.passing_side or "unknown",
            "completion_percentage": (self.gates_passed / self.total_gates) * 100 if self.total_gates > 0 else 0
        }
        
    def run(self):
        """Execute one cycle of the mission"""
        if not self.submarine_pose:
            rospy.logwarn_throttle(1, "Waiting for submarine pose...")
            return
            
        self.update_detected_objects()
        
        # Publish status
        status_msg = String()
        status_msg.data = f"Slalom Mission: Gate {self.current_gate + 1}/{self.total_gates}, " \
                         f"Passed {self.gates_passed}, Side: {self.passing_side or 'unknown'}"
        self.status_pub.publish(status_msg)
        
        if self.mission_tree_root and not self.mission_tree_root.completed:
            self.mission_tree_root.execute()
        else:
            rospy.loginfo("Slalom mission completed!")
            self.completed = True