#!/usr/bin/env python3
import sys
import types
from unittest import mock

# Mock ROS modules
print("Setting up ROS mocks...")

# Mock rospy
rospy_mock = mock.MagicMock()
rospy_mock.loginfo = lambda msg: print(f"[INFO] {msg}")
rospy_mock.logwarn = lambda msg: print(f"[WARN] {msg}")
rospy_mock.logerr = lambda msg: print(f"[ERROR] {msg}")
rospy_mock.is_shutdown = mock.Mock(return_value=False)
rospy_mock.signal_shutdown = mock.Mock()
rospy_mock.Publisher = mock.Mock()
rospy_mock.Rate = mock.Mock()
rospy_mock.init_node = mock.Mock()
rospy_mock.wait_for_service = mock.Mock()
rospy_mock.ServiceProxy = mock.Mock()
rospy_mock.ROSException = Exception
sys.modules['rospy'] = rospy_mock

# Mock geometry_msgs
geometry_msgs_mock = mock.MagicMock()
sys.modules['geometry_msgs'] = geometry_msgs_mock
sys.modules['geometry_msgs.msg'] = geometry_msgs_mock.msg

# Mock std_msgs  
std_msgs_mock = mock.MagicMock()
sys.modules['std_msgs'] = std_msgs_mock
sys.modules['std_msgs.msg'] = std_msgs_mock.msg

# Mock autonomy modules
autonomy_mock = mock.MagicMock()
sys.modules['autonomy'] = autonomy_mock
sys.modules['autonomy.srv'] = autonomy_mock.srv
sys.modules['autonomy.msg'] = autonomy_mock.msg

# Real Point and PoseStamped classes
class Point:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y  
        self.z = z

class PoseStamped:
    def __init__(self):
        self.header = type('', (), {'stamp': None, 'frame_id': 'map'})()
        self.pose = type('', (), {'position': Point(), 'orientation': Point()})()

# Set geometry_msgs classes
geometry_msgs_mock.msg.Point = Point
geometry_msgs_mock.msg.PoseStamped = PoseStamped

# Mock mission planner
mission_planner_mock = mock.MagicMock()
sys.modules['mission_planner'] = mission_planner_mock

# BaseMission class
class BaseMission:
    def __init__(self, name="base"):
        self.name = name
        self.detected_objects = {}
        self.submarine_pose = None
        self.active = True
        self.completed = False
        self.controller_client = None
    
    def create_action(self, task, obj_cls):
        return lambda: True
    
    def count_total_tasks(self): pass
    def search_mission_object(self, name): return self.detected_objects.get(name)
    def update_detected_objects(self): pass
    def deactivate(self): self.active = False
    def execute_identify_target(self, target_type): return True

# Set mission planner modules
base_mission_module = types.ModuleType('mission_planner.base_mission')
base_mission_module.BaseMission = BaseMission
sys.modules['mission_planner.base_mission'] = base_mission_module

# Mock types
from enum import Enum, auto
class TaskType(Enum):
    SEARCH = auto()
    APPROACH_TARGET = auto() 
    IDENTIFY_TARGET = auto()
    FIRE_TORPEDO = auto()
    SURFACE = auto()

types_module = types.ModuleType('mission_planner.types')
types_module.TaskType = TaskType
sys.modules['mission_planner.types'] = types_module

# Mock mission tree
class MissionTreeNode:
    def __init__(self, task, name=""):
        self.task = task
        self.name = name
        self.action = None
        self.children = []
        self.completed = False
    
    def add_child(self, node): 
        self.children.append(node)
    
    def execute(self): 
        # Don't complete immediately
        return False  # Let mission handle completion

tree_module = types.ModuleType('mission_planner.mission_tree')
tree_module.MissionTreeNode = MissionTreeNode
sys.modules['mission_planner.mission_tree'] = tree_module

print("ROS mocks set up successfully!")

import math
import time
from typing import Optional

# Mission object
class MissionObject:
    def __init__(self, position, name, confidence):
        self.position = position
        self.name = name
        self.confidence = confidence

# Import mission
from tagging_mission import TaggingMission

class TaggingTester:
    def __init__(self):
        self.mission = TaggingMission()
        self.mission.controller_client = self.create_mock_controller()
        self.mission.active = True
        self.mission.set_target_animal("reef_shark")
        
        self.board_pos = Point(x=8.0, y=0.0, z=-1.0)
        self.start_time = time.time()
        self.torpedoes_fired = 0
        self.phases_completed = {
            "board_detected": False,
            "config_detected": False,
            "distance_detected": False,
            "openings_detected": False,
            "first_torpedo": False,
            "second_torpedo": False
        }
        
        self.mission.fire_torpedo_service = self.mock_torpedo_service
        self.mission.completed = False
        
    def create_mock_controller(self):
        """Controller mock"""
        class Controller:
            def send_goal(self, *args, **kwargs): pass
            def wait_for_result(self, timeout=None): return True
            def get_result(self): 
                result = type('', (), {})()
                result.success = True
                return result
        return Controller()
    
    def mock_torpedo_service(self, torpedo_num):
        """Mock torpedo firing"""
        self.torpedoes_fired += 1
        result = type('', (), {})()
        
        if torpedo_num == 1:
            result.success = True
            result.message = "Torpedo 1 fired - full hit through opening"
            self.phases_completed["first_torpedo"] = True
            print(f"[TORPEDO] {result.message}")
        elif torpedo_num == 2:
            result.success = True  
            result.message = "Torpedo 2 fired - partial hit on board"
            self.phases_completed["second_torpedo"] = True
            print(f"[TORPEDO] {result.message}")
        else:
            result.success = False
            result.message = "No more torpedoes available"
            
        return result
    
    def get_submarine_pose(self):
        """Simulate submarine position"""
        t = time.time() - self.start_time
        pose = PoseStamped()
        
        # Move towards board
        progress = min(t / 10.0, 1.0)  # 10 seconds to board
        pose.pose.position.x = self.board_pos.x * progress * 0.7  # Stop before board
        pose.pose.position.y = 0.1 * math.sin(t)  # Small movement
        pose.pose.position.z = -0.5
        
        return pose
    
    def get_detected_object(self, name):
        """Simulate object detection"""
        t = time.time() - self.start_time
        
        # Progressive detection
        if name == "board" and t > 1.0:
            return MissionObject(self.board_pos, "board", 1.0)
        elif name == "reef_shark" and t > 2.0:
            pos = Point(self.board_pos.x, self.board_pos.y + 0.1, self.board_pos.z + 0.3)
            return MissionObject(pos, "reef_shark", 1.0)
        elif name == "sawfish" and t > 2.0:
            pos = Point(self.board_pos.x, self.board_pos.y - 0.1, self.board_pos.z - 0.3)
            return MissionObject(pos, "sawfish", 1.0)
        elif name == "opening_1" and t > 3.0:
            pos = Point(self.board_pos.x, self.board_pos.y + 0.15, self.board_pos.z + 0.25)
            return MissionObject(pos, "opening_1", 1.0)
        elif name == "opening_2" and t > 3.0:
            pos = Point(self.board_pos.x, self.board_pos.y - 0.15, self.board_pos.z - 0.25)
            return MissionObject(pos, "opening_2", 1.0)
        elif name == "horizontal_bar_1" and t > 1.5:
            pos = Point(self.board_pos.x - 3.0, -1.0, -1.0)
            return MissionObject(pos, "horizontal_bar_1", 1.0)
        
        return None
    
    def run(self):
        """Run the test"""
        print("Starting Tagging Mission Test")
        print("=" * 50)
        
        last_phase = None
        
        for i in range(600):  # 60 seconds at 10Hz
            t = time.time() - self.start_time
            
            # Update mission
            self.mission.submarine_pose = self.get_submarine_pose()
            self.mission.search_mission_object = self.get_detected_object
            
            # Manual phase progression and torpedo firing
            self.update_mission_phases(t)
            
            # Run mission (but don't let it complete via tree)
            original_completed = self.mission.completed
            self.mission.run()
            self.mission.completed = original_completed  # Restore our control
            
            # Log phase changes
            if self.mission.current_phase != last_phase:
                print(f"\nPhase: {self.mission.current_phase}")
                last_phase = self.mission.current_phase
            
            # Status every 3 seconds
            if i % 30 == 0:
                pos = self.mission.submarine_pose.pose.position
                print(f"Time: {t:.1f}s | Pos: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}) | "
                      f"Target: {self.mission.target_animal} | "
                      f"Torpedoes: {self.torpedoes_fired}/2 | "
                      f"Score: {self.mission.score['total']}")
            
            time.sleep(0.1)  # 10Hz
            
            # Check if mission should complete
            if self.phases_completed["second_torpedo"] and t > 15:
                self.mission.completed = True
                break
                
            if t > 60:  # Timeout
                break
        
        # Final results
        self.mission.calculate_final_score()
        
        print("\n" + "=" * 50)
        print("MISSION COMPLETE")
        print("=" * 50)
        print(f"Success: {self.mission.completed}")
        print(f"Target: {self.mission.target_animal}")
        print(f"Torpedoes: {self.torpedoes_fired}/2")
        print(f"Duration: {time.time() - self.start_time:.1f}s")
        print(f"Final Score: {self.mission.score['total']}")
        print("\nScore Breakdown:")
        for key, value in self.mission.score.items():
            if value > 0:
                print(f"   {key.replace('_', ' ').title()}: {value}")
        print("=" * 50)
    
    def update_mission_phases(self, t):
        """Manually progress mission phases and trigger actions"""
        # Board detection
        if t > 2 and not self.phases_completed["board_detected"]:
            self.phases_completed["board_detected"] = True
            print("Board detected!")
            
        # Configuration detection
        if t > 4 and not self.phases_completed["config_detected"]:
            self.mission.detect_board_configuration()
            self.phases_completed["config_detected"] = True
            print(f"Board configuration: {self.mission.board_configuration}")
            
        # Distance detection  
        if t > 6 and not self.phases_completed["distance_detected"]:
            self.mission.detect_far_distance()
            self.phases_completed["distance_detected"] = True
            print(f"Far distance: {self.mission.far_distance}")
            
        # Openings detection
        if t > 8 and not self.phases_completed["openings_detected"]:
            self.mission.detect_openings()
            self.phases_completed["openings_detected"] = True
            print("Openings detected!")
            
        # Fire first torpedo
        if t > 12 and not self.phases_completed["first_torpedo"]:
            self.mission.fire_torpedo()
            
        # Fire second torpedo  
        if t > 16 and not self.phases_completed["second_torpedo"] and self.phases_completed["first_torpedo"]:
            self.mission.fire_torpedo()

if __name__ == "__main__":
    tester = TaggingTester()
    try:
        tester.run()
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()