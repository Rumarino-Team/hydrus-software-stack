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
from typing import Dict, Optional, List, Tuple
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
    
    # Board dimensions from requirements
    BOARD_WIDTH = 0.6  # 2 ft in meters
    BOARD_HEIGHT = 0.6  # 2 ft in meters
    
    def __init__(self):
        super().__init__(name="Tagging")
        
        # Configure class names for detection
        self.cls_names = {
            "reef_shark": "reef_shark",
            "sawfish": "sawfish",
            "board": "board",
            "opening": "opening",  # For detecting torpedo openings
            "horizontal_bar": "horizontal_bar"  # For detecting far distance markers
        }
        
        # Task-specific parameters
        self.target_animal = "reef_shark"  # Default target - will be updated from gate mission
        self.second_animal = "sawfish"     # The other animal to target after the first
        
        # Board configuration
        self.board_configuration = None  # Will be "reef_shark_top" or "sawfish_top"
        self.openings_detected = []  # List of detected openings with positions
        self.target_opening = None  # Opening near the chosen animal
        self.second_opening = None  # Opening near the other animal
        
        # Distance parameters
        self.far_distance = None  # Will be detected from horizontal bars
        self.default_far_distance = 3.0  # Default if bars not detected
        self.current_distance = 0.0  # Current distance from board
        
        # Scoring system
        self.score = {
            "full_hits": 0,  # Torpedoes through openings
            "partial_hits": 0,  # Torpedoes touching board
            "correct_animal_bonus": 0,  # Hit near chosen animal
            "order_bonus": 0,  # Correct order (chosen first)
            "distance_bonus": 0,  # Fired from far distance
            "total": 0
        }
        
        # Hit tracking
        self.first_target_hit = False
        self.first_hit_type = None  # "full", "partial", or None
        self.second_target_hit = False
        self.second_hit_type = None
        self.current_target = self.target_animal
        self.torpedo_index = 0
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
            "Detect Board Configuration",
            "Detect Far Distance",
            "Approach Board",
            "Identify Target Opening",
            "Fire First Torpedo",
            "Identify Second Opening",
            "Fire Second Torpedo",
            "Calculate Score",
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
        
        # Phase 2: Detect board configuration and components
        detect_config = MissionTreeNode(task=TaskType.IDENTIFY_TARGET, name="Detect Board Configuration")
        detect_config.action = self.create_action(TaskType.IDENTIFY_TARGET, "board_configuration")
        search_board.add_child(detect_config)
        
        # Phase 3: Detect far distance markers
        detect_distance = MissionTreeNode(task=TaskType.IDENTIFY_TARGET, name="Detect Far Distance")
        detect_distance.action = self.create_action(TaskType.IDENTIFY_TARGET, "horizontal_bar")
        detect_config.add_child(detect_distance)
        
        # Phase 4: Move to optimal firing position
        approach_board = MissionTreeNode(task=TaskType.APPROACH_TARGET, name="Approach Board")
        approach_board.action = self.create_action(TaskType.APPROACH_TARGET, "board")
        detect_distance.add_child(approach_board)
        
        # Phase 5: Identify first target opening
        identify_first = MissionTreeNode(task=TaskType.IDENTIFY_TARGET, name=f"Identify {self.target_animal} Opening")
        identify_first.action = self.create_action(TaskType.IDENTIFY_TARGET, "target_opening")
        approach_board.add_child(identify_first)
        
        # Phase 6: Fire first torpedo
        fire_first = MissionTreeNode(task=TaskType.FIRE_TORPEDO, name=f"Fire at {self.target_animal} Opening")
        fire_first.action = self.create_action(TaskType.FIRE_TORPEDO, "target_opening")
        identify_first.add_child(fire_first)
        
        # Phase 7: Identify second target opening
        identify_second = MissionTreeNode(task=TaskType.IDENTIFY_TARGET, name=f"Identify {self.second_animal} Opening")
        identify_second.action = self.create_action(TaskType.IDENTIFY_TARGET, "second_opening")
        fire_first.add_child(identify_second)
        
        # Phase 8: Fire second torpedo
        fire_second = MissionTreeNode(task=TaskType.FIRE_TORPEDO, name=f"Fire at {self.second_animal} Opening")
        fire_second.action = self.create_action(TaskType.FIRE_TORPEDO, "second_opening")
        identify_second.add_child(fire_second)
        
        # Phase 9: Surface after task completion
        surface = MissionTreeNode(task=TaskType.SURFACE, name="Surface")
        surface.action = self.create_action(TaskType.SURFACE, None)
        fire_second.add_child(surface)
        
        return root
    
    def detect_board_configuration(self) -> bool:
        """Detect which animal is on top of the board"""
        rospy.loginfo("[Tagging] Detecting board configuration")
        
        # Update detected objects
        self.update_detected_objects()
        
        # Look for animals and their positions
        reef_shark = self.search_mission_object("reef_shark")
        sawfish = self.search_mission_object("sawfish")
        
        if reef_shark and sawfish:
            # Compare z-coordinates (assuming z is vertical)
            if reef_shark.position.z > sawfish.position.z:
                self.board_configuration = "reef_shark_top"
            else:
                self.board_configuration = "sawfish_top"
            
            rospy.loginfo(f"[Tagging] Board configuration: {self.board_configuration}")
            return True
        
        rospy.loginfo("[Tagging] Cannot determine board configuration yet")
        return False
    
    def detect_far_distance(self) -> bool:
        """Detect the horizontal bars that indicate far distance"""
        rospy.loginfo("[Tagging] Detecting far distance markers")
        
        # Look for horizontal bars
        bars = []
        for obj_name, obj in self.detected_objects.items():
            if "horizontal_bar" in obj_name:
                bars.append(obj)
        
        if bars:
            # Calculate distance based on bar positions
            board = self.search_mission_object("board")
            if board:
                # The far distance is the distance from the board to the bars
                bar_distances = [self.calculate_distance(bar.position, board.position) for bar in bars]
                self.far_distance = max(bar_distances)  # Use the farthest bar
                rospy.loginfo(f"[Tagging] Far distance detected: {self.far_distance:.2f}m")
                return True
        
        # Use default if bars not detected
        self.far_distance = self.default_far_distance
        rospy.loginfo(f"[Tagging] Using default far distance: {self.far_distance:.2f}m")
        return True
    
    def detect_openings(self) -> bool:
        """Detect the two openings on the board"""
        rospy.loginfo("[Tagging] Detecting openings")
        
        # Look for openings
        self.openings_detected = []
        for obj_name, obj in self.detected_objects.items():
            if "opening" in obj_name:
                self.openings_detected.append(obj)
        
        if len(self.openings_detected) >= 2:
            rospy.loginfo(f"[Tagging] Found {len(self.openings_detected)} openings")
            
            # Determine which opening is near which animal
            reef_shark = self.search_mission_object("reef_shark")
            sawfish = self.search_mission_object("sawfish")
            
            if reef_shark and sawfish:
                # Find closest opening to each animal
                for opening in self.openings_detected:
                    dist_to_shark = self.calculate_distance(opening.position, reef_shark.position)
                    dist_to_sawfish = self.calculate_distance(opening.position, sawfish.position)
                    
                    if dist_to_shark < dist_to_sawfish:
                        if self.target_animal == "reef_shark":
                            self.target_opening = opening
                        else:
                            self.second_opening = opening
                    else:
                        if self.target_animal == "sawfish":
                            self.target_opening = opening
                        else:
                            self.second_opening = opening
                
                return True
        
        rospy.loginfo("[Tagging] Not enough openings detected")
        return False
    
    def calculate_distance(self, pos1: Point, pos2: Point) -> float:
        """Calculate distance between two points"""
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        dz = pos1.z - pos2.z
        return (dx**2 + dy**2 + dz**2)**0.5
    
    def execute_identify_target(self, target_type: str) -> bool:
        """Identify various targets on the board"""
        if target_type == "board_configuration":
            return self.detect_board_configuration()
        elif target_type == "horizontal_bar":
            return self.detect_far_distance()
        elif target_type in ["target_opening", "second_opening"]:
            return self.detect_openings()
        else:
            # Default behavior for other targets
            return super().execute_identify_target(target_type)
    
    def fire_torpedo(self) -> bool:
        """Fire a torpedo and track the result"""
        if self.torpedo_index >= self.max_torpedoes:
            rospy.logwarn("[Tagging] No torpedoes left to fire")
            return False
        
        torpedo_number = self.torpedo_index + 1
        rospy.loginfo(f"[Tagging] Firing torpedo {torpedo_number}")
        
        # Check if we're at far distance for bonus
        board = self.search_mission_object("board")
        if board and self.submarine_pose:
            sub_pos = self.submarine_pose.pose.position
            self.current_distance = self.calculate_distance(sub_pos, board.position)
            is_far = self.current_distance >= self.far_distance
        else:
            is_far = False
        
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
                rospy.loginfo(f"[Tagging] Torpedo {torpedo_number} fired: {response.message}")
                
                # Parse the response to determine hit type
                # This assumes the service returns hit information in the message
                hit_type = self.parse_hit_result(response.message)
                
                # Update hit tracking and scoring
                if self.torpedo_index == 0:  # First torpedo
                    self.first_target_hit = True
                    self.first_hit_type = hit_type
                    
                    # Update scores
                    if hit_type == "full":
                        self.score["full_hits"] += 1
                        if self.current_target == self.target_animal:
                            self.score["correct_animal_bonus"] += 1
                            self.score["order_bonus"] += 1  # Correct order
                    elif hit_type == "partial":
                        self.score["partial_hits"] += 1
                    
                    if is_far:
                        self.score["distance_bonus"] += 1
                    
                else:  # Second torpedo
                    self.second_target_hit = True
                    self.second_hit_type = hit_type
                    
                    # Update scores
                    if hit_type == "full":
                        self.score["full_hits"] += 1
                        if self.current_target == self.target_animal:
                            self.score["correct_animal_bonus"] += 1
                    elif hit_type == "partial":
                        self.score["partial_hits"] += 1
                    
                    if is_far:
                        self.score["distance_bonus"] += 1
                
                # Update torpedo count and target
                self.torpedo_index += 1
                self.current_target = self.second_animal
                
                return True
            else:
                rospy.logerr(f"[Tagging] Failed to fire torpedo {torpedo_number}: {response.message}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"[Tagging] Service call failed: {e}")
            return False
    
    def parse_hit_result(self, message: str) -> str:
        """Parse the hit result from the torpedo service response"""
        # This is a placeholder - actual implementation depends on service response format
        if "through opening" in message.lower() or "full hit" in message.lower():
            return "full"
        elif "hit board" in message.lower() or "partial" in message.lower():
            return "partial"
        else:
            return "miss"
    
    def calculate_final_score(self):
        """Calculate the final score based on all factors"""
        # Define point values (these should match the competition rules)
        FULL_HIT_POINTS = 100
        PARTIAL_HIT_POINTS = 50
        CORRECT_ANIMAL_BONUS = 50
        ORDER_BONUS = 25
        DISTANCE_BONUS = 25
        
        self.score["total"] = (
            self.score["full_hits"] * FULL_HIT_POINTS +
            self.score["partial_hits"] * PARTIAL_HIT_POINTS +
            self.score["correct_animal_bonus"] * CORRECT_ANIMAL_BONUS +
            self.score["order_bonus"] * ORDER_BONUS +
            self.score["distance_bonus"] * DISTANCE_BONUS
        )
        
        rospy.loginfo(f"[Tagging] Final Score: {self.score['total']}")
        rospy.loginfo(f"[Tagging] Score Breakdown: {self.score}")
    
    def update_phase(self):
        """Update the current mission phase"""
        if self.current_phase_index < len(self.phases):
            old_phase = self.current_phase
            self.current_phase = self.phases[self.current_phase_index]
            
            if old_phase != self.current_phase:
                rospy.loginfo(f"[Tagging] Phase: {self.current_phase}")
            
            # Advance phases based on completion
            phase_complete = False
            
            if self.current_phase == "Search for Board" and "board" in self.detected_objects:
                phase_complete = True
            elif self.current_phase == "Detect Board Configuration" and self.board_configuration:
                phase_complete = True
            elif self.current_phase == "Detect Far Distance" and self.far_distance:
                phase_complete = True
            elif self.current_phase == "Approach Board" and self.current_distance >= self.far_distance - 0.1:
                phase_complete = True
            elif self.current_phase == "Identify Target Opening" and self.target_opening:
                phase_complete = True
            elif self.current_phase == "Fire First Torpedo" and self.first_target_hit:
                phase_complete = True
            elif self.current_phase == "Identify Second Opening" and self.second_opening:
                phase_complete = True
            elif self.current_phase == "Fire Second Torpedo" and self.second_target_hit:
                phase_complete = True
            elif self.current_phase == "Calculate Score":
                self.calculate_final_score()
                phase_complete = True
            
            if phase_complete:
                self.current_phase_index += 1
    
    def set_target_animal(self, animal: str):
        """Set the primary target animal from previous mission data"""
        if animal in ["reef_shark", "sawfish"]:
            self.target_animal = animal
            self.second_animal = "sawfish" if animal == "reef_shark" else "reef_shark"
            self.current_target = self.target_animal
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
                rospy.loginfo(f"[Tagging] Final Score: {self.score['total']}")
                self.completed = True
                self.deactivate()
        else:
            rospy.logwarn("[Tagging] No mission tree defined")
            self.deactivate()
            