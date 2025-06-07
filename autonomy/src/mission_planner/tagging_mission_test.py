import sys
import types
import unittest
from unittest import mock
from enum import Enum, auto

# Mock ROS modules
sys.modules['rospy'] = mock.MagicMock()
sys.modules['std_msgs'] = mock.MagicMock()
sys.modules['std_msgs.msg'] = mock.MagicMock()
sys.modules['geometry_msgs'] = mock.MagicMock()
sys.modules['geometry_msgs.msg'] = mock.MagicMock()
sys.modules['autonomy'] = mock.MagicMock()
sys.modules['autonomy.srv'] = mock.MagicMock()

# Create proper mission_planner module structure
sys.modules['mission_planner'] = mock.MagicMock()

# Create a proper BaseMission class
class MockBaseMission:
    def __init__(self, name="base"):
        self.name = name
        self.detected_objects = {}
        self.submarine_pose = None
        self.active = True
        self.completed = False
        self.controller_client = mock.Mock()
        
    def create_action(self, task, obj_cls):
        return lambda: True
        
    def count_total_tasks(self):
        pass
        
    def search_mission_object(self, name):
        return self.detected_objects.get(name)
        
    def update_detected_objects(self):
        pass
        
    def deactivate(self):
        self.active = False
        
    def execute_identify_target(self, target_type):
        return True

# Set up the modules
base_mission_module = types.ModuleType('mission_planner.base_mission')
base_mission_module.BaseMission = MockBaseMission
sys.modules['mission_planner.base_mission'] = base_mission_module

# Mock types
types_module = types.ModuleType('mission_planner.types')
class TaskType(Enum):
    SEARCH = auto()
    APPROACH_TARGET = auto()
    IDENTIFY_TARGET = auto()
    FIRE_TORPEDO = auto()
    SURFACE = auto()
types_module.TaskType = TaskType
sys.modules['mission_planner.types'] = types_module

# Mock mission tree
tree_module = types.ModuleType('mission_planner.mission_tree')
class MissionTreeNode:
    def __init__(self, task, name=""):
        self.task = task
        self.name = name
        self.action = None
        self.children = []
    def add_child(self, node):
        self.children.append(node)
tree_module.MissionTreeNode = MissionTreeNode
sys.modules['mission_planner.mission_tree'] = tree_module

# Import tagging_mission
from tagging_mission import TaggingMission

class TestTaggingMission(unittest.TestCase):
    def setUp(self):
        self.tm = TaggingMission()
        
        # Mock torpedo service with hit result
        def mock_torpedo_service(torpedo_num):
            if torpedo_num == 1:
                return types.SimpleNamespace(
                    success=True, 
                    message="Torpedo 1 fired - full hit through opening"
                )
            else:
                return types.SimpleNamespace(
                    success=True, 
                    message="Torpedo 2 fired - partial hit on board"
                )
        
        self.tm.fire_torpedo_service = mock_torpedo_service
        
    def test_default_configuration(self):
        """Test default mission configuration"""
        self.assertEqual(self.tm.target_animal, "reef_shark")
        self.assertEqual(self.tm.second_animal, "sawfish")
        self.assertEqual(self.tm.current_target, "reef_shark")
        self.assertEqual(self.tm.max_torpedoes, 2)
        
    def test_set_target_animal(self):
        """Test changing target animal"""
        self.tm.set_target_animal("sawfish")
        self.assertEqual(self.tm.target_animal, "sawfish")
        self.assertEqual(self.tm.second_animal, "reef_shark")
        self.assertEqual(self.tm.current_target, "sawfish")
        
        # Check mission tree is rebuilt
        tree_names = []
        def collect_names(node):
            tree_names.append(node.name)
            for child in node.children:
                collect_names(child)
        collect_names(self.tm.mission_tree_root)
        self.assertIn("Identify sawfish Opening", tree_names)
        
    def test_scoring_system_initialized(self):
        """Test scoring system is properly initialized"""
        expected_keys = ["full_hits", "partial_hits", "correct_animal_bonus", 
                        "order_bonus", "distance_bonus", "total"]
        for key in expected_keys:
            self.assertIn(key, self.tm.score)
            self.assertEqual(self.tm.score[key], 0)
            
    def test_phases_defined(self):
        """Test mission phases are defined"""
        self.assertIn("Search for Board", self.tm.phases)
        self.assertIn("Detect Board Configuration", self.tm.phases)
        self.assertIn("Fire First Torpedo", self.tm.phases)
        
    def test_mission_tree_structure(self):
        """Test mission tree is built"""
        self.assertIsNotNone(self.tm.mission_tree_root)
        self.assertEqual(self.tm.mission_tree_root.name, "Root")
        
    def test_board_dimensions_exist(self):
        """Test board dimensions - check if they exist in the module"""
        import tagging_mission
        self.assertTrue(hasattr(tagging_mission.TaggingMission, 'BOARD_WIDTH'))
        self.assertTrue(hasattr(tagging_mission.TaggingMission, 'BOARD_HEIGHT'))
        if hasattr(tagging_mission.TaggingMission, 'BOARD_WIDTH'):
            self.assertEqual(tagging_mission.TaggingMission.BOARD_WIDTH, 0.6)
        if hasattr(tagging_mission.TaggingMission, 'BOARD_HEIGHT'):
            self.assertEqual(tagging_mission.TaggingMission.BOARD_HEIGHT, 0.6)

    def test_board_configuration_detection(self):
        """Test board configuration detection"""
        # Mock detected objects
        mock_shark = mock.Mock(position=mock.Mock(x=0, y=0, z=1.0))
        mock_sawfish = mock.Mock(position=mock.Mock(x=0, y=0, z=0.5))
        
        self.tm.detected_objects = {
            "reef_shark": mock_shark,
            "sawfish": mock_sawfish
        }
        
        result = self.tm.detect_board_configuration()
        self.assertTrue(result)
        self.assertEqual(self.tm.board_configuration, "reef_shark_top")
        
        # Test reverse configuration
        mock_shark.position.z = 0.5
        mock_sawfish.position.z = 1.0
        result = self.tm.detect_board_configuration()
        self.assertTrue(result)
        self.assertEqual(self.tm.board_configuration, "sawfish_top")

    def test_far_distance_detection(self):
        """Test detection of far distance from horizontal bars"""
        # Mock board and horizontal bars
        mock_board = mock.Mock(position=mock.Mock(x=0, y=0, z=0))
        mock_bar = mock.Mock(position=mock.Mock(x=3.5, y=0, z=0))
        
        self.tm.detected_objects = {
            "board": mock_board,
            "horizontal_bar_1": mock_bar
        }
        self.tm.search_mission_object = lambda n: self.tm.detected_objects.get(n)
        
        result = self.tm.detect_far_distance()
        self.assertTrue(result)
        self.assertAlmostEqual(self.tm.far_distance, 3.5, places=1)
        
        # Test default distance when no bars detected
        self.tm.detected_objects = {"board": mock_board}
        result = self.tm.detect_far_distance()
        self.assertTrue(result)
        self.assertEqual(self.tm.far_distance, self.tm.default_far_distance)

    def test_opening_detection(self):
        """Test detection of openings and association with animals"""
        # Mock detected objects
        mock_shark = mock.Mock(position=mock.Mock(x=0, y=0, z=1.0))
        mock_sawfish = mock.Mock(position=mock.Mock(x=0, y=0, z=-1.0))
        mock_opening1 = mock.Mock(position=mock.Mock(x=0, y=0, z=0.9))  # Near shark
        mock_opening2 = mock.Mock(position=mock.Mock(x=0, y=0, z=-0.9))  # Near sawfish
        
        self.tm.detected_objects = {
            "reef_shark": mock_shark,
            "sawfish": mock_sawfish,
            "opening_1": mock_opening1,
            "opening_2": mock_opening2
        }
        self.tm.search_mission_object = lambda n: self.tm.detected_objects.get(n)
        
        # Test with reef_shark as target
        self.tm.target_animal = "reef_shark"
        result = self.tm.detect_openings()
        self.assertTrue(result)
        self.assertEqual(self.tm.target_opening, mock_opening1)
        self.assertEqual(self.tm.second_opening, mock_opening2)
        
        # Test with sawfish as target
        self.tm.target_animal = "sawfish"
        self.tm.target_opening = None
        self.tm.second_opening = None
        result = self.tm.detect_openings()
        self.assertTrue(result)
        self.assertEqual(self.tm.target_opening, mock_opening2)
        self.assertEqual(self.tm.second_opening, mock_opening1)

    def test_hit_result_parsing(self):
        """Test parsing of torpedo hit results"""
        self.assertEqual(self.tm.parse_hit_result("full hit through opening"), "full")
        self.assertEqual(self.tm.parse_hit_result("Torpedo hit board partially"), "partial")
        self.assertEqual(self.tm.parse_hit_result("Torpedo missed"), "miss")

    def test_fire_torpedo_with_scoring(self):
        """Test torpedo firing with score tracking"""
        # Setup submarine position for distance calculation
        self.tm.submarine_pose = mock.Mock(
            pose=mock.Mock(position=mock.Mock(x=3.5, y=0, z=0))
        )
        mock_board = mock.Mock(position=mock.Mock(x=0, y=0, z=0))
        self.tm.detected_objects = {"board": mock_board}
        self.tm.search_mission_object = lambda n: self.tm.detected_objects.get(n)
        self.tm.far_distance = 3.0
        
        # Fire first torpedo (should be full hit based on mock)
        self.tm.current_target = self.tm.target_animal
        result = self.tm.fire_torpedo()
        self.assertTrue(result)
        self.assertEqual(self.tm.torpedo_index, 1)
        self.assertTrue(self.tm.first_target_hit)
        self.assertEqual(self.tm.first_hit_type, "full")
        self.assertEqual(self.tm.score["full_hits"], 1)
        self.assertEqual(self.tm.score["correct_animal_bonus"], 1)
        self.assertEqual(self.tm.score["order_bonus"], 1)
        self.assertEqual(self.tm.score["distance_bonus"], 1)  # Far distance
        
        # Fire second torpedo (should be partial hit based on mock)
        result = self.tm.fire_torpedo()
        self.assertTrue(result)
        self.assertEqual(self.tm.torpedo_index, 2)
        self.assertTrue(self.tm.second_target_hit)
        self.assertEqual(self.tm.second_hit_type, "partial")
        self.assertEqual(self.tm.score["partial_hits"], 1)
        self.assertEqual(self.tm.score["distance_bonus"], 2)  # Both from far

    def test_final_score_calculation(self):
        """Test final score calculation"""
        # Set up score components
        self.tm.score = {
            "full_hits": 2,
            "partial_hits": 0,
            "correct_animal_bonus": 2,
            "order_bonus": 1,
            "distance_bonus": 2,
            "total": 0
        }
        
        self.tm.calculate_final_score()
        
        # Expected: 2*100 + 0*50 + 2*50 + 1*25 + 2*25 = 375
        self.assertEqual(self.tm.score["total"], 375)

if __name__ == "__main__":
    unittest.main()