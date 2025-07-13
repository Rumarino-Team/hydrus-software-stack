#!/usr/bin/env python3
# mission_planner/gate_mission.py
"""
Task 1 - Collecting Data (Gate) Mission
This task consists of a gate with a red divider and images of a Reef Shark and a Sawfish.
The submarine must choose a marine animal by passing under that side of the gate.
Additional points are awarded for passing through with style (orientation changes).
"""

import rospy
import math
import numpy as np
from typing import Dict, Optional, List, Tuple
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Import from mission planner
from mission_planner.base_mission import BaseMission
from mission_planner.types import TaskType, MissionObject
from mission_planner.mission_tree import MissionTreeNode
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointGoal

class GateMission(BaseMission):
    """
    Implements the Task 1 - Collecting Data (Gate) mission.
    The submarine must detect a gate with a red divider, choose to pass under either
    the Reef Shark or Sawfish side, and navigate through with style for additional points.
    """
    _REQUIRED_FRAMES = 2
    def __init__(self):
        super().__init__(name="Collecting Data (Gate)")
        
        # Configure class names for detection
        self.cls_names = {
            0: "gate",           # Main gate structure
            1: "gate_divider",   # Red divider in the middle
            2: "reef_shark",     # Reef Shark image
            3: "sawfish",        # Sawfish image
            4: "red_box",        # Red box (top-right)
            5: "black_box"       # Black box
        }
        
        # Configuration for the mission
        self.chosen_animal = "reef_shark"  # Default target animal
        self.gate_width = 3.0  # 120 inches = 3 meters
        self.gate_height = 1.5  # 60 inches = 1.5 meters
        self.divider_position = None  # Will be updated when detected
        
        # Style options for passing through gate
        self.style_completed = False
        self.current_style = None  # None, "yaw", "roll", "pitch"
        self.style_points = 0
        self.previous_orientations = []  # Track previous orientations to avoid duplicates
        
        # Gate detected and passed flags
        self.gate_detected = False
        self.gate_passed = False
        
        # Shark and sawfish positions
        self.shark_side = None  # "left" or "right" of divider
        self.sawfish_side = None  # "left" or "right" of divider
        
        # Initialize phases
        self.current_phase = "Initialization"
        self.phases = [
            "Search for Gate",
            "Identify Images",
            "Choose Side",
            "Approach Gate",
            "Navigate with Style",
            "Gate Passed",
            "Mission Complete"
        ]
        self.current_phase_index = 0
        self._gate_seen_streak = 0
        
        # Build the mission tree
        self.mission_tree_root = self.build_mission_tree()
        self.count_total_tasks()
    
    def update_phase(self):
        """Single, clean phase-transition block."""
        # 1. refresh detection status first
        detection = self.search_mission_object("gate") is not None

        if detection:
           self._gate_seen_streak += 1
        else:
            self._gate_seen_streak = 0
        self.gate_detected = (self._gate_seen_streak >= self._REQUIRED_FRAMES)
        # 2. decide what phase we’re in
        old_phase = self.current_phase
        self.current_phase = self.phases[self.current_phase_index]

        # 3. advance when conditions are met
        if self.current_phase == "Search for Gate" and detection:
            self.current_phase_index += 1
        elif self.current_phase == "Identify Images" and self.shark_side and self.sawfish_side:
            self.current_phase_index += 1
        elif self.current_phase == "Choose Side" and self.chosen_animal:
            self.current_phase_index += 1
        elif self.current_phase == "Approach Gate" and self.is_positioned_for_gate():
            self.current_phase_index += 1
        elif self.current_phase == "Navigate with Style" and self.gate_passed:
            self.current_phase_index += 1
        elif self.current_phase == "Gate Passed" and self.style_completed:
            self.current_phase_index += 1
            self.completed = True

        # 4. log any phase change
        if self.current_phase != old_phase:
            rospy.loginfo(f"[Gate] Phase: {self.current_phase}")

    def build_mission_tree(self) -> MissionTreeNode:
        """Build the mission execution tree"""
        # Root node
        root = MissionTreeNode(task=TaskType.SEARCH, name="Root")
        
        # Phase 1: Search for the gate
        search_gate = MissionTreeNode(task=TaskType.SEARCH, name="Search for Gate")
        search_gate.action = self.create_action(TaskType.SEARCH, "gate")
        root.add_child(search_gate)
        
        # Phase 2: Identify images on the gate
        identify_images = MissionTreeNode(task=TaskType.IDENTIFY_TARGET, name="Identify Images")
        identify_images.action = self.create_search_images_action()
        search_gate.add_child(identify_images)
        
        # Phase 3: Choose which animal to target
        choose_side = MissionTreeNode(task=TaskType.SEARCH, name="Choose Side")
        choose_side.action = self.create_choose_side_action()
        identify_images.add_child(choose_side)
        
        # Phase 4: Approach the gate
        approach_gate = MissionTreeNode(task=TaskType.MOVE_TO_CENTER, name="Approach Gate")
        approach_gate.action = self.create_approach_gate_action()
        choose_side.add_child(approach_gate)
        
        # Phase 5: Navigate through gate with style
        navigate_gate = MissionTreeNode(task=TaskType.MOVE_TO_CENTER, name="Navigate Gate with Style")
        navigate_gate.action = self.create_navigate_with_style_action()
        approach_gate.add_child(navigate_gate)
        
        # Phase 6: Surface after task completion
        surface = MissionTreeNode(task=TaskType.SURFACE, name="Surface")
        surface.action = self.create_action(TaskType.SURFACE, None)
        navigate_gate.add_child(surface)
        
        return root
    
    def create_search_images_action(self):
        """Create an action to search for and identify the images on the gate"""
        def action():
            self.current_task_name = "Identify Images"
            self.current_object_name = "gate"
            
            # Update detected objects
            self.update_detected_objects()
            
            # Check if we have detected the reef shark and sawfish images
            reef_shark_obj = self.search_mission_object("reef_shark")
            sawfish_obj = self.search_mission_object("sawfish")
            divider_obj = self.search_mission_object("gate_divider")
            
            if not divider_obj:
                rospy.loginfo("[Gate] Searching for divider...")
                return False
                
            self.divider_position = divider_obj.position
            
            if not reef_shark_obj or not sawfish_obj:
                rospy.loginfo("[Gate] Still searching for marine animal images...")
                return False
            
            # Determine which side each animal is on
            # Left/right determined by looking at the gate from the front
            if reef_shark_obj.position.y < divider_obj.position.y:
                self.shark_side = "left"
                self.sawfish_side = "right"
            else:
                self.shark_side = "right"
                self.sawfish_side = "left"
                
            rospy.loginfo(f"[Gate] Reef Shark identified on {self.shark_side} side")
            rospy.loginfo(f"[Gate] Sawfish identified on {self.sawfish_side} side")
            
            return True
            
        return action
    
    def create_choose_side_action(self):
        """Create an action to choose which animal to target"""
        def action():
            self.current_task_name = "Choose Side"
            self.current_object_name = "gate"
            
            # For simplicity, we'll choose the reef shark by default.
            # This could be more sophisticated, based on mission requirements.
            self.chosen_animal = "reef_shark"
            
            rospy.loginfo(f"[Gate] Chosen to pass under {self.chosen_animal} side")
            return True
            
        return action
    
    def create_approach_gate_action(self):
        """Generate the callable that will be run by the mission tree"""
        def action():
            rospy.loginfo("=== APPROACH VALIDATION ===")
    
            # 1 ─ valid animal?
            valid_animals = ("reef_shark", "sawfish")
            if self.chosen_animal not in valid_animals:
                rospy.logerr(f"[Gate] Invalid animal '{self.chosen_animal}'")
                return False
    
            # 2 ─ sides found?
            if not (self.shark_side and self.sawfish_side):
                rospy.logerr("[Gate] Animal sides not determined")
                return False
    
            # 3 ─ required objects visible?
            for name in ("gate", "gate_divider", self.chosen_animal):
                if not self.search_mission_object(name):
                    rospy.logerr(f"[Gate] Missing required object: {name}")
                    return False
    
            # 4 ─ pose available?
            if not self.submarine_pose:
                rospy.logerr("[Gate] No submarine pose available")
                return False
    
            # 5 ─ correct side of divider?
            current_y  = self.submarine_pose.pose.position.y
            divider_y  = self.divider_position.y
            target_side = self.shark_side if self.chosen_animal == "reef_shark" else self.sawfish_side
            tol = 0.1
            if (target_side == "left" and current_y > divider_y + tol) or \
               (target_side == "right" and current_y < divider_y - tol):
                rospy.logerr("[Gate] Wrong side of the divider")
                return False
    
            # 6 ─ move to approach point (unchanged)
            gate_obj = self.search_mission_object("gate")
            approach_y_offset = -0.5 if (self.chosen_animal == "reef_shark" and self.shark_side == "left") \
                               or (self.chosen_animal == "sawfish" and self.sawfish_side == "left") \
                            else 0.5
            approach_point = Point(
                x=gate_obj.position.x - 2.0,
                y=self.divider_position.y + approach_y_offset,
                z=self.divider_position.z - 0.5
            )
            goal = NavigateToWaypointGoal(target_point=approach_point)
            self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)
            self.controller_client.wait_for_result()
            return bool(self.controller_client.get_result() and
                        self.controller_client.get_result().success)
    
        return action  # <- the mission tree now gets the callable
    
    def create_navigate_with_style_action(self):
        """Create an action to navigate through the gate with style (orientation changes)"""
        def action():
            self.current_task_name = "Navigate with Style"
            self.current_object_name = "gate"
            
            gate_obj = self.search_mission_object("gate")
            if not gate_obj:
                rospy.logwarn("[Gate] Gate not detected")
                return False
            
            # If we haven't started the style navigation yet, pick a style
            if not self.current_style:
                # Choose a style: roll, pitch, or yaw
                # Roll and pitch are worth more points
                styles = ["roll", "pitch", "yaw"]
                self.current_style = np.random.choice(styles, p=[0.4, 0.4, 0.2])  # Higher probability for roll and pitch
                rospy.loginfo(f"[Gate] Navigating through gate with style: {self.current_style}")
                
                # Save current orientation as starting point
                if self.submarine_pose:
                    quat = [
                        self.submarine_pose.pose.orientation.x,
                        self.submarine_pose.pose.orientation.y,
                        self.submarine_pose.pose.orientation.z,
                        self.submarine_pose.pose.orientation.w
                    ]
                    euler = euler_from_quaternion(quat)
                    self.previous_orientations.append(euler)
            
            # Calculate a point beyond the gate to navigate to
            target_point = Point()
            target_point.x = gate_obj.position.x + 2.0  # 2 meters beyond the gate
            
            # Y position based on chosen animal side
            approach_y_offset = 0.5  # Offset from center to chosen side (in meters)
            if self.chosen_animal == "reef_shark":
                if self.shark_side == "left":
                    approach_y_offset = -approach_y_offset  # Move left
            elif self.chosen_animal == "sawfish":
                if self.sawfish_side == "left":
                    approach_y_offset = -approach_y_offset  # Move left
                    
            target_point.y = self.divider_position.y + approach_y_offset
            target_point.z = self.divider_position.z - 0.5  # Below the divider
            
            # Apply style orientation
            if self.current_style and self.submarine_pose:
                quat = [
                    self.submarine_pose.pose.orientation.x,
                    self.submarine_pose.pose.orientation.y,
                    self.submarine_pose.pose.orientation.z,
                    self.submarine_pose.pose.orientation.w
                ]
                euler = list(euler_from_quaternion(quat))
                
                # Create 90-degree rotation based on chosen style
                if self.current_style == "roll":
                    euler[0] += math.pi/2  # Add 90° to roll
                elif self.current_style == "pitch":
                    euler[1] += math.pi/2  # Add 90° to pitch
                elif self.current_style == "yaw":
                    euler[2] += math.pi/2  # Add 90° to yaw
                
                # Convert back to quaternion
                quat = quaternion_from_euler(euler[0], euler[1], euler[2])
                
                # Apply orientation to goal
                goal = NavigateToWaypointGoal()
                goal.target_point = target_point
                goal.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                
                self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)
                self.controller_client.wait_for_result()
                result = self.controller_client.get_result()
                
                # Record this orientation for style points
                if not self.is_duplicate_orientation(euler):
                    self.previous_orientations.append(euler)
                    self.style_points += 1 if self.current_style == "yaw" else 2  # More points for roll and pitch
                    rospy.loginfo(f"[Gate] Gained {self.style_points} style points")
            else:
                # No style, just navigate through
                goal = NavigateToWaypointGoal()
                goal.target_point = target_point
                self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)
                self.controller_client.wait_for_result()
                result = self.controller_client.get_result()
            
            if result and result.success:
                rospy.loginfo(f"[Gate] Successfully navigated through gate")
                self.gate_passed = True
                self.style_completed = True
                return True
            else:
                rospy.logwarn(f"[Gate] Failed to navigate through gate")
                return False
            
        return action
    
    def is_duplicate_orientation(self, new_orientation, tolerance=math.pi/4):
        """Check if the orientation is a duplicate (within tolerance) of a previous one"""
        for prev_orientation in self.previous_orientations:
            # Check if all euler angles are close enough to be considered duplicates
            if all(abs(n - p) < tolerance for n, p in zip(new_orientation, prev_orientation)):
                return True
        return False
    
    def is_positioned_for_gate(self):
        """Check if the submarine is positioned at the approach point"""
        gate_obj = self.search_mission_object("gate")
        if not gate_obj or not self.submarine_pose:
            return False
        
        # Calculate distance to gate
        dx = gate_obj.position.x - self.submarine_pose.pose.position.x
        dy = gate_obj.position.y - self.submarine_pose.pose.position.y
        dz = gate_obj.position.z - self.submarine_pose.pose.position.z
        
        distance = (dx**2 + dy**2 + dz**2)**0.5
        
        # Check if we're about 2 meters in front of the gate
        return 1.5 <= distance <= 2.5
    def _update_gate_detection(self):
        """Recalculate gate_detected every tick with simple debouncing."""
        if self.search_mission_object("gate"):
            self._gate_seen_streak += 1
        else:
            self._gate_seen_streak = 0

        self.gate_detected = self._gate_seen_streak >= self._REQUIRED_FRAMES
        

        # If we lost the gate, roll back to the first phase
              
    
    def get_status(self) -> dict:
        """Return the current status of the mission"""
        status = super().get_status()
        status.update({
            "gate_detected": self.gate_detected,
            "gate_passed": self.gate_passed,
            "chosen_animal": self.chosen_animal,
            "style_points": self.style_points,
            "style_used": self.current_style
        })
        return status
    
    def run(self):
        """Run one iteration of the mission"""
        if not self.active:
            return
        
        # Update phase
        self.update_phase()
        
        # Update detected objects
        self.update_detected_objects()
        
        # If we have a mission tree, run it
        if self.mission_tree_root:
            # Execute the first incomplete node
            if self.mission_tree_root.execute():
                rospy.loginfo(f"[Gate] Mission complete! Chosen animal: {self.chosen_animal}, Style points: {self.style_points}")
                self.completed = True
                self.deactivate()
        else:
            rospy.logwarn("[Gate] No mission tree defined")
            self.deactivate()
    
    def get_chosen_animal(self):
        """Return the chosen animal for subsequent missions"""
        return self.chosen_animal