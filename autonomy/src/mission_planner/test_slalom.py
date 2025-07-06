import unittest
import rospy
import rostest
from geometry_msgs.msg import Point, PoseStamped
from autonomy.msg import Detection, Detections
from mission_planner.slalom_mission import SlalomMission


class TestSlalomMission(unittest.TestCase):
    def setUp(self):
        # Initialize ROS node for testing
        if not rospy.get_node_uri():
            rospy.init_node('test_slalom_mission', anonymous=True)
        
        # Create mission instance with ROS environment
        self.mission = SlalomMission(enable_action_client=False)
        
        # Wait a bit for ROS to initialize
        rospy.sleep(0.1)
        
    def tearDown(self):
        # Clean up after each test
        self.mission = None

    def make_detection(self, cls, x, y, z):
        det = Detection()
        det.cls = cls
        det.point = Point(x=x, y=y, z=z)
        return det

    def test_detect_gate_sides_success(self):
        # 1 red pipe at (1,0,0), 2 white pipes at (1,-1,0) and (1,1,0)
        detections = Detections()
        detections.detections = [
            self.make_detection(2, 1, 0, 0),    # Red
            self.make_detection(1, 1, -1, 0),   # White left
            self.make_detection(1, 1, 1, 0)     # White right
        ]
        self.mission.current_detections = detections
        gate_sides = self.mission.detect_gate_sides(0)
        self.assertIn('left_side', gate_sides)
        self.assertIn('right_side', gate_sides)
        self.assertIn('center', gate_sides)
        self.assertAlmostEqual(gate_sides['center'].y, 0.0, places=2)

    def test_detect_gate_sides_insufficient_detections(self):
        detections = Detections()
        detections.detections = [
            self.make_detection(2, 1, 0, 0),    # Only red
            self.make_detection(1, 1, -1, 0)    # Only one white
        ]
        self.mission.current_detections = detections
        gate_sides = self.mission.detect_gate_sides(0)
        self.assertEqual(gate_sides, {})

    def test_detect_gate_sides_no_detections(self):
        self.mission.current_detections = Detections()
        gate_sides = self.mission.detect_gate_sides(0)
        self.assertEqual(gate_sides, {})

    def test_detect_gate_success_and_update(self):
        # First detection
        detections = Detections()
        detections.detections = [
            self.make_detection(2, 1, 0, 0),
            self.make_detection(1, 1, -1, 0),
            self.make_detection(1, 1, 1, 0)
        ]
        self.mission.current_detections = detections
        result = self.mission.detect_gate(0)
        self.assertTrue(result)
        self.assertEqual(len(self.mission.gate_positions), 1)
        # Update detection (simulate new detection at same gate)
        result2 = self.mission.detect_gate(0)
        self.assertTrue(result2)
        self.assertEqual(self.mission.gate_positions[0]['detection_count'], 2)

    def test_detect_gate_fail(self):
        # Not enough detections
        detections = Detections()
        detections.detections = [
            self.make_detection(2, 1, 0, 0)
        ]
        self.mission.current_detections = detections
        result = self.mission.detect_gate(0)
        self.assertFalse(result)

    def test_navigate_gate_simulated(self):
        # Prepare a gate position
        gate = {
            'target_point': Point(x=1, y=2, z=3)
        }
        self.mission.gate_positions = [gate]
        self.mission.gates_passed = 0
        self.mission.gates_completed = []
        result = self.mission.navigate_gate(0)
        self.assertTrue(result)
        self.assertEqual(self.mission.gates_passed, 1)
        self.assertIn(0, self.mission.gates_completed)

    def test_navigate_gate_fail(self):
        # No gate position
        self.mission.gate_positions = []
        result = self.mission.navigate_gate(0)
        self.assertFalse(result)

    def test_increment_gate(self):
        self.mission.current_gate = 0
        self.mission.total_gates = 2
        result = self.mission.increment_gate()
        self.assertTrue(result)
        self.assertEqual(self.mission.current_gate, 1)
        # Increment again, should still return True and log completion
        result2 = self.mission.increment_gate()
        self.assertTrue(result2)
        self.assertEqual(self.mission.current_gate, 2)

    def test_get_status(self):
        self.mission.gates_passed = 2
        self.mission.current_gate = 1
        self.mission.total_gates = 3
        self.mission.gate_positions = [{}, {}]
        status = self.mission.get_status()
        self.assertEqual(status['gates_passed'], 2)
        self.assertEqual(status['current_gate'], 1)
        self.assertEqual(status['total_gates'], 3)
        self.assertEqual(status['gates_detected'], 2)
        self.assertAlmostEqual(status['completion_percentage'], 66.666, places=1)

    def test_ros_integration_detection_callback(self):
        """Test that the ROS detection callback works"""
        # Create test detection message
        detections = Detections()
        detections.detections = [
            self.make_detection(2, 1, 0, 0),
            self.make_detection(1, 1, -1, 0),
            self.make_detection(1, 1, 1, 0)
        ]
        
        # Call the callback directly
        self.mission.detection_callback(detections)
        
        # Verify the detection was stored
        self.assertIsNotNone(self.mission.current_detections)
        self.assertEqual(len(self.mission.current_detections.detections), 3)

    def test_ros_integration_pose_callback(self):
        """Test that the ROS pose callback works"""
        # Create test pose message
        pose = PoseStamped()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 3.0
        
        # Call the callback directly
        self.mission.pose_callback(pose)
        
        # Verify the pose was stored
        self.assertIsNotNone(self.mission.submarine_pose)
        self.assertEqual(self.mission.submarine_pose.pose.position.x, 1.0)
        self.assertEqual(self.mission.submarine_pose.pose.position.y, 2.0)
        self.assertEqual(self.mission.submarine_pose.pose.position.z, 3.0)


if __name__ == '__main__':
    rostest.rosrun('mission_planner', 'test_slalom_mission', TestSlalomMission)