#!/usr/bin/env python3

import rospy
import math
from typing import List, Dict
from geometry_msgs.msg import Point, PoseStamped
from autonomy.msg import Detections
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointGoal, NavigateToWaypointFeedback, NavigateToWaypointResult
from std_msgs.msg import String
import actionlib

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
    
    def __init__(self, enable_action_client=True):
        # Initialize the base mission
        super().__init__(name="Slalom Navigation")
        
        # Configuration specific to Slalom
        self.cls_names = {1: 'WhitePipe', 2: 'RedPipe'}
        
        # Slalom specific parameters
        self.passing_side = "left"  # Always pass on left side of red pipe
        self.gates_passed = 0
        self.total_gates = 3
        self.current_gate = 0
        self.gate_positions: List[Dict[str, Point]] = []
        self.gates_completed = []
        
        # Initialize detection and pose storage
        self.current_detections = None
        self.submarine_pose = None
        
        # Additional subscribers/publishers
        self.status_pub = rospy.Publisher('/mission_status', String, queue_size=10)
        self.detection_sub = rospy.Subscriber('/vision/detections', Detections, self.detection_callback)
        self.pose_sub = rospy.Subscriber('/submarine/pose', PoseStamped, self.pose_callback)
        
        # Action client for navigation (optional for testing)
        self.controller_client = None
        if enable_action_client:
            self.controller_client = actionlib.SimpleActionClient('/navigate_to_waypoint', NavigateToWaypointAction)
            rospy.loginfo("[Slalom Navigation] Waiting for action server to start...")
            self.controller_client.wait_for_server()
            rospy.loginfo("[Slalom Navigation] Action server started.")
        
        # Waypoints for search pattern
        self.waypoints = []
        self.current_waypoint_index = 0
        
        # Build the mission tree
        self.mission_tree_root = self.build_mission_tree()
        
    def detection_callback(self, msg: Detections):
        """Callback for handling detection messages."""
        self.current_detections = msg

    def pose_callback(self, msg: PoseStamped):
        """Callback for handling pose messages."""
        self.submarine_pose = msg
        
    def feedback_callback(self, feedback: NavigateToWaypointFeedback):
        """Callback for navigation feedback."""
        pass
        
    def detect_gate_sides(self, gate_number: int) -> dict:
        """
        Detect the left and right sides of a gate relative to the red pipe.
        Returns a dictionary with 'left_side', 'right_side', and 'center' points.
        """
        if not self.current_detections or not self.current_detections.detections:
            rospy.logwarn(f"No detections available for gate {gate_number}")
            return {}
            
        # Find red and white pipes
        red_pipes = []
        white_pipes = []
        
        for detection in self.current_detections.detections:
            if detection.cls == 2:  # Red pipe
                red_pipes.append(detection)
            elif detection.cls == 1:  # White pipe
                white_pipes.append(detection)
        
        # Need at least one red pipe (center) and two white pipes (sides)
        if len(red_pipes) < 1 or len(white_pipes) < 2:
            rospy.logwarn(f"Insufficient detections for gate {gate_number}: {len(red_pipes)} red, {len(white_pipes)} white")
            return {}
            
        # Use the closest red pipe (assuming it's the center of the gate)
        red_pipe = min(red_pipes, key=lambda p: abs(p.point.x))
        center_y = red_pipe.point.y
        
        # Sort white pipes by y-coordinate relative to center
        white_pipes.sort(key=lambda p: p.point.y)
        
        # Find white pipes on either side of the red pipe
        left_pipes = [p for p in white_pipes if p.point.y < center_y]
        right_pipes = [p for p in white_pipes if p.point.y > center_y]
        
        if not left_pipes or not right_pipes:
            rospy.logwarn(f"Could not find pipes on both sides for gate {gate_number}")
            return {}
        
        # Use the closest white pipes to the red pipe
        left_pipe = max(left_pipes, key=lambda p: p.point.y)  # Rightmost of left pipes
        right_pipe = min(right_pipes, key=lambda p: p.point.y)  # Leftmost of right pipes
        
        # Calculate gate sides - points between red pipe and white pipes
        gate_sides = {}
        
        # Left side (where submarine should pass)
        gate_sides['left_side'] = Point(
            x=(red_pipe.point.x + left_pipe.point.x) / 2,
            y=(red_pipe.point.y + left_pipe.point.y) / 2,
            z=(red_pipe.point.z + left_pipe.point.z) / 2
        )
        
        # Right side (for reference)
        gate_sides['right_side'] = Point(
            x=(red_pipe.point.x + right_pipe.point.x) / 2,
            y=(red_pipe.point.y + right_pipe.point.y) / 2,
            z=(red_pipe.point.z + right_pipe.point.z) / 2
        )
        
        # Store the red pipe position as center reference
        gate_sides['center'] = Point(
            x=red_pipe.point.x,
            y=red_pipe.point.y,
            z=red_pipe.point.z
        )
        
        rospy.loginfo(f"Gate {gate_number + 1} sides detected - Left: ({gate_sides['left_side'].y:.2f}), "
                     f"Center: ({gate_sides['center'].y:.2f}), Right: ({gate_sides['right_side'].y:.2f})")
        
        return gate_sides
        
    def detect_gate(self, gate_number: int) -> bool:
        """Detect if both pipes of a gate are visible and store/update gate sides"""
        if not self.current_detections:
            return False
            
        # Use the new gate sides detection
        gate_sides = self.detect_gate_sides(gate_number)
        
        if gate_sides:
            # Always update gate position (even if already exists)
            if gate_number >= len(self.gate_positions):
                # First time detection - append new gate
                self.gate_positions.append({
                    'left_side': gate_sides['left_side'],
                    'right_side': gate_sides['right_side'],
                    'center': gate_sides['center'],
                    'target_point': gate_sides['left_side'],
                    'detection_count': 1,
                    'last_updated': rospy.Time.now()
                })
                rospy.loginfo(f"Gate {gate_number + 1} detected for first time")
            else:
                # Update existing gate position with new detection
                old_pos = self.gate_positions[gate_number]['center']
                self.gate_positions[gate_number].update({
                    'left_side': gate_sides['left_side'],
                    'right_side': gate_sides['right_side'],
                    'center': gate_sides['center'],
                    'target_point': gate_sides['left_side'],
                    'detection_count': self.gate_positions[gate_number].get('detection_count', 0) + 1,
                    'last_updated': rospy.Time.now()
                })
                new_pos = gate_sides['center']
                distance_moved = ((old_pos.x - new_pos.x)**2 + (old_pos.y - new_pos.y)**2 + (old_pos.z - new_pos.z)**2)**0.5
                rospy.loginfo(f"Gate {gate_number + 1} position updated (moved {distance_moved:.2f}m)")
            
            return True
        return False
        
    def determine_passing_side(self) -> str:
        """Determine which side the submarine should pass on (always left for this mission)"""
        return "left"  # Always pass on the left side of the red pipe
        
    def navigate_gate(self, gate_number: int) -> bool:
        """Navigate through the left side of the gate (left of red pipe)"""
        if gate_number >= len(self.gate_positions):
            rospy.logwarn(f"No gate position stored for gate {gate_number}")
            return False
            
        # Get the gate position
        gate = self.gate_positions[gate_number]
        
        # Always use the left side target point
        target_point = gate['target_point']
        
        # Add some offset to ensure we pass through the left side clearly
        adjusted_target = Point()
        adjusted_target.x = target_point.x
        adjusted_target.y = target_point.y - 0.3  # Additional 0.3m offset to the left
        adjusted_target.z = target_point.z
        
        rospy.loginfo(f"Navigating through left side of gate {gate_number + 1} at "
                     f"({adjusted_target.x:.2f}, {adjusted_target.y:.2f}, {adjusted_target.z:.2f})")
        
        # Navigate to this point (if action client is available)
        if self.controller_client:
            goal = NavigateToWaypointGoal()
            goal.target_point = adjusted_target
            self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)
            self.controller_client.wait_for_result()
            result = self.controller_client.get_result()
            
            # If navigation was successful, mark this gate as passed
            if result and result.success:
                self.gates_passed += 1
                self.gates_completed.append(gate_number)
                rospy.loginfo(f"Successfully passed gate {gate_number + 1}")
                return True
            return False
        else:
            # For testing without action client, just simulate success
            self.gates_passed += 1
            self.gates_completed.append(gate_number)
            rospy.loginfo(f"Successfully passed gate {gate_number + 1} (simulated)")
            return True

    def execute_search_for_gate(self) -> bool:
        """Search for the next gate in the slalom course"""
        rospy.loginfo(f"Searching for gate {self.current_gate + 1}...")
        
        # Simple search pattern - rotate in place
        if self.submarine_pose is None:
            rospy.logwarn("No submarine pose available for search")
            return False
            
        # Create a point to rotate around
        center = self.submarine_pose.pose.position
        
        # Generate rotation waypoints
        if not self.waypoints:
            for angle in range(0, 360, 45):  # 8 waypoints in a circle
                rad = math.radians(angle)
                waypoint = Point()
                waypoint.x = center.x + 2.0 * math.cos(rad)  # 2m radius
                waypoint.y = center.y + 2.0 * math.sin(rad)
                waypoint.z = center.z
                self.waypoints.append(waypoint)
            
        # Move to next waypoint in rotation (if action client is available)
        if self.current_waypoint_index < len(self.waypoints):
            if self.controller_client:
                goal = NavigateToWaypointGoal()
                goal.target_point = self.waypoints[self.current_waypoint_index]
                self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)
                self.controller_client.wait_for_result()
            else:
                rospy.loginfo(f"Searching at waypoint {self.current_waypoint_index + 1} (simulated)")
            
            self.current_waypoint_index += 1
            return True
                
        return False

    def build_mission_tree(self) -> MissionTreeNode:
        """Build the mission tree for the slalom mission"""
        # Main mission node
        root = MissionTreeNode(
            task=TaskType.HOLD_POSITION,
            name="Slalom Mission Start",
            action=lambda: True
        )
        
        # Configure task parameters for the mission
        if hasattr(self, 'task_params'):
            self.task_params.search_timeout = 30.0  # seconds
            self.task_params.hold_time = 3.0  # seconds
        
        # First Gate
        gate1_branch = MissionTreeNode(
            task=TaskType.HOLD_POSITION,
            name="Gate 1",
            action=lambda: True
        )
        
        search_gate1 = MissionTreeNode(
            task=TaskType.SEARCH,
            name="Search for Gate 1",
            condition=lambda: self.current_gate == 0 and not self.detect_gate(0),
            action=lambda: self.execute_search_for_gate()
        )
        
        navigate_gate1 = MissionTreeNode(
            task=TaskType.MOVE_TO_CENTER,
            name="Navigate Gate 1 Left Side",
            condition=lambda: self.current_gate == 0 and self.detect_gate(0),
            action=lambda: self.navigate_gate(0) and self.increment_gate()
        )
        
        gate1_branch.add_child(search_gate1)
        gate1_branch.add_child(navigate_gate1)
        root.add_child(gate1_branch)
        
        return root
    
    def increment_gate(self) -> bool:
        """Move to the next gate"""
        self.current_gate += 1
        if self.current_gate >= self.total_gates:
            rospy.loginfo("All gates completed!")
            return True
        return True
    
    def get_status(self) -> dict:
        """Get current mission status"""
        return {
            'gates_passed': self.gates_passed,
            'current_gate': self.current_gate,
            'total_gates': self.total_gates,
            'completion_percentage': (self.gates_passed / self.total_gates) * 100.0 if self.total_gates > 0 else 0.0,
            'gates_detected': len(self.gate_positions)
        }
        
    def run(self):
        """Run the slalom mission"""
        rospy.loginfo("Starting Slalom Mission")
        
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Execute mission tree
            if self.mission_tree_root:
                self.mission_tree_root.execute()
            
            # Check if mission is complete
            if self.gates_passed >= self.total_gates:
                rospy.loginfo("Slalom mission completed successfully!")
                break
                
            rate.sleep()
        
        rospy.loginfo("Slalom mission ended")