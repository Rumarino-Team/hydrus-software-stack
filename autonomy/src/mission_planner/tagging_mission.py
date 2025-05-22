#!/usr/bin/env python3
# mission_planner/tagging_mission.py
"""
Task 4 - Tagging (Torpedoes) Mission
This task consists of one vertical board with images of reef shark and sawfish,
with openings that the submarine must fire torpedoes through.
Points are awarded for firing torpedoes through any opening, with more points
for hitting the opening near the marine animal chosen in Collecting Data (Gate).
"""

import rospy
import time
from typing import Dict, Optional, List
from std_msgs.msg import Int16
from geometry_msgs.msg import Point

# Import from mission planner
from mission_planner.base_mission import BaseMission
from mission_planner.types import TaskType
from mission_planner.mission_tree import MissionTreeNode

# Import service for torpedo firing
from autonomy.srv import FireTorpedo

class TaggingMission(BaseMission):
    """
    Implements the Task 4 - Tagging (Torpedoes) mission.
    The submarine must fire torpedoes through openings on a board with images
    of a reef shark and sawfish.
    """
    
    def __init__(self):
        super().__init__(name="Tagging")
        
        # Configure class names for detection
        self.cls_names = {
            "reef_shark": "reef_shark",
            "sawfish": "sawfish",
            "board": "board"
        }
        
        # Task-specific parameters
        self.target_animal = "reef_shark"  # Could be updated from previous mission
        self.second_animal = "sawfish"     # The other animal to target after the first
        
        # Preferred firing distance - "far" distance for more points
        self.far_distance = 3.0  # meters from the board
        
        # Scores and tracking
        self.first_target_hit = False
        self.second_target_hit = False
        self.current_target = self.target_animal
        self.torpedo_index = 0  # Track which torpedo is being fired
        
        # Maximum torpedo count
        self.max_torpedoes = 2
        
        # Connect to the torpedo service
        self.torpedo_service_name = 'fire_torpedo'
        rospy.loginfo(f"[Tagging] Waiting for torpedo service '{self.torpedo_service_name}'...")
        try:
            rospy.wait_for_service(self.torpedo_service_name, timeout=5.0)
            self.fire_torpedo_service = rospy.ServiceProxy(self.torpedo_service_name, FireTorpedo)
            rospy.loginfo(f"[Tagging] Connected to torpedo service")
        except rospy.ROSException:
            rospy.logwarn(f"[Tagging] Torpedo service not available - will retry when needed")
            self.fire_torpedo_service = None
        
        # Build the mission tree
        self.mission_tree_root = self.build_mission_tree()
        self.count_total_tasks()
        
        # Initialize phases
        self.current_phase = "Initialization"
        self.phases = [
            "Search for Board",
            "Approach Board",
            "Identify First Target",
            "Fire First Torpedo",
            "Identify Second Target",
            "Fire Second Torpedo",
            "Mission Complete"
        ]
        self.current_phase_index = 0
    
    def build_mission_tree(self) -> MissionTreeNode:
        """Build the mission execution tree"""
        # Root node
        root = MissionTreeNode(task=TaskType.SEARCH, name="Root")
        
        # Phase 1: Search for the board
        search_board = MissionTreeNode(task=TaskType.SEARCH, name="Search for Board")
        search_board.action = self.create_action(TaskType.SEARCH, "board")
        root.add_child(search_board)
        
        # Phase 2: Move to and approach the board
        approach_board = MissionTreeNode(task=TaskType.APPROACH_TARGET, name="Approach Board")
        approach_board.action = self.create_action(TaskType.APPROACH_TARGET, "board")
        search_board.add_child(approach_board)
        
        # Phase 3: Identify first target (animal chosen in previous task)
        identify_first = MissionTreeNode(task=TaskType.IDENTIFY_TARGET, name=f"Identify {self.target_animal}")
        identify_first.action = self.create_action(TaskType.IDENTIFY_TARGET, self.target_animal)
        approach_board.add_child(identify_first)
        
        # Phase 4: Fire first torpedo
        fire_first = MissionTreeNode(task=TaskType.FIRE_TORPEDO, name=f"Fire at {self.target_animal}")
        fire_first.action = self.create_action(TaskType.FIRE_TORPEDO, self.target_animal)
        identify_first.add_child(fire_first)
        
        # Phase 5: Identify second target
        identify_second = MissionTreeNode(task=TaskType.IDENTIFY_TARGET, name=f"Identify {self.second_animal}")
        identify_second.action = self.create_action(TaskType.IDENTIFY_TARGET, self.second_animal)
        fire_first.add_child(identify_second)
        
        # Phase 6: Fire second torpedo
        fire_second = MissionTreeNode(task=TaskType.FIRE_TORPEDO, name=f"Fire at {self.second_animal}")
        fire_second.action = self.create_action(TaskType.FIRE_TORPEDO, self.second_animal)
        identify_second.add_child(fire_second)
        
        # Phase 7: Surface after task completion
        surface = MissionTreeNode(task=TaskType.SURFACE, name="Surface")
        surface.action = self.create_action(TaskType.SURFACE, None)
        fire_second.add_child(surface)
        
        return root
    
    def execute_identify_target(self, target_animal: str) -> bool:
        """Identify the target animal on the board"""
        rospy.loginfo(f"[Tagging] Identifying target: {target_animal}")
        
        # Check if we've already detected this animal
        target_obj = self.search_mission_object(target_animal)
        if target_obj:
            rospy.loginfo(f"[Tagging] Target {target_animal} already identified")
            return True
        
        # Update detected objects
        self.update_detected_objects()
        
        # Check if we have now detected the animal
        target_obj = self.search_mission_object(target_animal)
        if target_obj:
            rospy.loginfo(f"[Tagging] Found {target_animal} at distance {target_obj.distance:.2f}m")
            return True
        
        rospy.loginfo(f"[Tagging] {target_animal} not yet identified")
        return False
    
    def execute_approach_target(self, target_name: str) -> bool:
        """Approach the target to the optimal firing distance"""
        target_obj = self.search_mission_object(target_name)
        if not target_obj:
            rospy.logwarn(f"[Tagging] Cannot approach - {target_name} not found")
            return False
        
        # Calculate a position at the desired distance from the board
        if self.submarine_pose is None:
            rospy.logwarn("[Tagging] Cannot approach - submarine pose unknown")
            return False
        
        # Calculate direction vector from submarine to target
        dx = target_obj.position.x - self.submarine_pose.pose.position.x
        dy = target_obj.position.y - self.submarine_pose.pose.position.y
        dz = target_obj.position.z - self.submarine_pose.pose.position.z
        
        # Normalize the direction vector
        distance = (dx**2 + dy**2 + dz**2)**0.5
        if distance < 0.001:  # Avoid division by zero
            rospy.logwarn("[Tagging] Already at target position")
            return True
        
        dx /= distance
        dy /= distance
        dz /= distance
        
        # Calculate the desired position (far distance from target)
        desired_position = Point(
            x=target_obj.position.x - dx * self.far_distance,
            y=target_obj.position.y - dy * self.far_distance,
            z=target_obj.position.z - dz * self.far_distance
        )
        
        # Move to the desired position
        goal = NavigateToWaypointGoal()
        goal.target_point = desired_position
        self.controller_client.send_goal(goal, feedback_cb=self.feedback_callback)
        self.controller_client.wait_for_result()
        result = self.controller_client.get_result()
        
        if result and result.success:
            rospy.loginfo(f"[Tagging] Successfully positioned at {self.far_distance}m from {target_name}")
            return True
        else:
            rospy.logwarn(f"[Tagging] Failed to position at {self.far_distance}m from {target_name}")
            return False
    
    def fire_torpedo(self) -> bool:
        """Fire a torpedo using the torpedo service"""
        if self.torpedo_index >= self.max_torpedoes:
            rospy.logwarn("[Tagging] No torpedoes left to fire")
            return False
        
        torpedo_number = self.torpedo_index + 1  # 1-based torpedo number
        rospy.loginfo(f"[Tagging] Firing torpedo {torpedo_number} at {self.current_target}")
        
        # Ensure service connection
        if self.fire_torpedo_service is None:
            try:
                rospy.wait_for_service(self.torpedo_service_name, timeout=2.0)
                self.fire_torpedo_service = rospy.ServiceProxy(self.torpedo_service_name, FireTorpedo)
            except rospy.ROSException:
                rospy.logerr(f"[Tagging] Torpedo service not available")
                return False
        
        # Call the torpedo firing service
        try:
            response = self.fire_torpedo_service(torpedo_number)
            if response.success:
                rospy.loginfo(f"[Tagging] Torpedo {torpedo_number} fired successfully: {response.message}")
                
                # Increment torpedo index
                self.torpedo_index += 1
                
                # Mark the appropriate target as hit
                if self.current_target == self.target_animal:
                    self.first_target_hit = True
                    self.current_target = self.second_animal
                else:
                    self.second_target_hit = True
                
                return True
            else:
                rospy.logerr(f"[Tagging] Failed to fire torpedo {torpedo_number}: {response.message}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"[Tagging] Service call failed: {e}")
            return False
    
    def create_action(self, task: TaskType, object_cls: Optional[str]) -> callable:
        """Create an action function for a task"""
        base_action = super().create_action(task, object_cls)
        
        def enhanced_action():
            self.current_task_name = task.name
            self.current_object_name = object_cls if object_cls is not None else "N/A"
            
            # Add custom task handling for torpedo-specific tasks
            if task == TaskType.IDENTIFY_TARGET:
                return self.execute_identify_target(object_cls)
            elif task == TaskType.APPROACH_TARGET:
                return self.execute_approach_target(object_cls)
            elif task == TaskType.FIRE_TORPEDO:
                return self.fire_torpedo()
            else:
                # Use the base implementation for other task types
                return base_action()
                
        return enhanced_action
    
    def update_phase(self):
        """Update the current mission phase"""
        if self.current_phase_index < len(self.phases):
            old_phase = self.current_phase
            self.current_phase = self.phases[self.current_phase_index]
            
            if old_phase != self.current_phase:
                rospy.loginfo(f"[Tagging] Phase: {self.current_phase}")
                
            # Advance to next phase based on mission progress
            if self.current_phase == "Search for Board" and "board" in self.detected_objects:
                self.current_phase_index += 1
            elif self.current_phase == "Approach Board" and self.search_mission_object("board") and \
                 self.search_mission_object("board").distance <= self.far_distance + 0.5:
                self.current_phase_index += 1
            elif self.current_phase == "Identify First Target" and self.search_mission_object(self.target_animal):
                self.current_phase_index += 1
            elif self.current_phase == "Fire First Torpedo" and self.first_target_hit:
                self.current_phase_index += 1
            elif self.current_phase == "Identify Second Target" and self.search_mission_object(self.second_animal):
                self.current_phase_index += 1
            elif self.current_phase == "Fire Second Torpedo" and self.second_target_hit:
                self.current_phase_index += 1
    
    def set_target_animal(self, animal: str):
        """Set the primary target animal from previous mission data"""
        if animal in ["reef_shark", "sawfish"]:
            self.target_animal = animal
            self.second_animal = "sawfish" if animal == "reef_shark" else "reef_shark"
            rospy.loginfo(f"[Tagging] Primary target set to {self.target_animal}")
            
            # Rebuild the mission tree with the new targets
            self.mission_tree_root = self.build_mission_tree()
            self.count_total_tasks()
    
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
                rospy.loginfo("[Tagging] Mission complete!")
                self.completed = True
                self.deactivate()
        else:
            rospy.logwarn("[Tagging] No mission tree defined")
            self.deactivate()

# For importing from mission_manager.py
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointGoal