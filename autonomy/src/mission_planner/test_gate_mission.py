#!/usr/bin/env python3
# filepath: /catkin_ws/src/hydrus-software-stack/autonomy/src/mission_planner/test_gate_mission.py
import math
import unittest
from types import SimpleNamespace

import rospy
from mission_planner.gate_mission import GateMission
from geometry_msgs.msg import PoseStamped, Point


# ────────────────────────── helpers ──────────────────────────
class _Obj:
    def __init__(self, x=10.0, y=0.0, z=10.0):
        self.position = SimpleNamespace(x=x, y=y, z=z)


GATE = _Obj()
DIVIDER = _Obj()
SHARK = _Obj(y=-0.5)
SAWFISH = _Obj(y=+0.5)


class TestGateMissionScenarios(unittest.TestCase):
    # ---------- fake SimpleActionClient ----------
    @staticmethod
    def _make_pose(x=0.0, y=0.0, z=10.0):
        # returns an object with .pose.position.x/y/z—exactly what GateMission uses
        return SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z)
            )
        )

    @staticmethod
    def _stub_action_client():
        class _Stub:
            def send_goal(self, *_, **__):
                pass

            def wait_for_result(self, timeout=None):
                return True

            def get_result(self):
                return SimpleNamespace(success=True)

            def wait_for_server(self, timeout=None):
                return True

            def cancel_all_goals(self):
                pass

        return _Stub()

    @classmethod
    def setUpClass(cls):
        if not rospy.core.is_initialized():
            rospy.init_node("gate_mission_unittests",
                            anonymous=True,
                            disable_signals=True)

    # 1 ─ full happy-path
    def test_phase_progression_success(self):
        m = GateMission()
        m.controller_client = self._stub_action_client()

        # Pretend the sharks were identified
        m.shark_side = "left"
        m.sawfish_side = "right"

        def _search(name):
            return {
                "gate": GATE,
                "gate_divider": DIVIDER,
                "reef_shark": SHARK,
                "sawfish": SAWFISH
            }.get(name)

        m.search_mission_object = _search
        m.is_positioned_for_gate = lambda: True

        for _ in range(10):
            m.update_phase()
            if m.current_phase == "Navigate with Style":
                m.gate_passed = True
            if m.current_phase == "Gate Passed":
                m.style_completed = True

        self.assertTrue(m.completed)
        self.assertEqual(m.current_phase, "Mission Complete")

    # 2 ─ gate disappears
    def test_gate_disappears_during_search(self):
        m = GateMission()
        m.controller_client = self._stub_action_client()
        m.search_mission_object = lambda name: None
        m.update_phase()
        self.assertEqual(m.current_phase, "Search for Gate")
        self.assertFalse(m.gate_detected)

    # 3 ─ only gate
    def test_only_gate_detected(self):
        m = GateMission()
        m.controller_client = self._stub_action_client()
        m.search_mission_object = lambda name: GATE if name == "gate" else None
        m.update_phase()
        m.update_phase()
        self.assertEqual(m.current_phase, "Identify Images")
        self.assertIsNone(getattr(m, "shark_side", None))
        self.assertIsNone(getattr(m, "sawfish_side", None))

    # 4 ─ divider but no gate
    def test_divider_without_gate(self):
        m = GateMission()
        m.controller_client = self._stub_action_client()
        m.search_mission_object = lambda name: DIVIDER if name == "gate_divider" else None
        m.update_phase()
        self.assertEqual(m.current_phase, "Search for Gate")
        self.assertFalse(m.gate_detected)

    # 5 ─ invalid animal selection
    def test_incorrect_animal_side_selection(self):
        # ---------- common mission setup ----------
        m = GateMission()
        m.controller_client = self._stub_action_client()
        m.chosen_animal = "dolfin"  # unknown animal
        m.shark_side = "left"
        m.sawfish_side = "right"
        m.search_mission_object = lambda name: GATE if name == "gate" else DIVIDER if name == "gate_divider" else None
        m.submarine_pose = self._make_pose()
        self.assertFalse(
            m.create_approach_gate_action()(),
            "approach_action should not be called before the gate is detected"
        )

    # 6 ─ navigate-with-style happy-path
    def test_navigate_with_style_success(self):
        """Gate detected, style chosen ('roll'), controller reports success."""
        from types import SimpleNamespace
        from unittest.mock import patch

        # ── helper: fake action goal with the right, **mutable** fields ──
        class FakeNavigateGoal:
            __slots__ = ("target_point", "orientation")
            def __init__(self):
                self.target_point = None
                self.orientation = None

        m = GateMission()
        m.controller_client = self._stub_action_client()

        # —— minimal mission state required by the action ——
        m.divider_position = DIVIDER.position
        m.search_mission_object = lambda name: GATE if name == "gate" else None
        m.submarine_pose = SimpleNamespace(
            pose=SimpleNamespace(
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
            )
        )
        m.is_duplicate_orientation = lambda *_: False
        m.previous_orientations = []
        m.style_points = 0

        m.chosen_animal = "reef_shark"
        m.shark_side = "left"

        with patch("mission_planner.gate_mission.NavigateToWaypointGoal", FakeNavigateGoal), \
             patch("mission_planner.gate_mission.np.random.choice", return_value="roll"):
            ok = m.create_navigate_with_style_action()()

        # —— assertions ——
        self.assertTrue(ok)
        self.assertTrue(m.gate_passed)
        self.assertTrue(m.style_completed)
        self.assertEqual(m.current_style, "roll")
        self.assertEqual(m.style_points, 2)          # roll ⇒ +2 points
        self.assertEqual(len(m.previous_orientations), 2)

    def test_is_duplicate_orientation(self):
        """Test detection of duplicate orientations during style navigation."""
        m = GateMission()

        # Add a starting orientation (roll=0, pitch=0, yaw=0)
        initial_orientation = [0.0, 0.0, 0.0]
        m.previous_orientations.append(initial_orientation)

        # Test 1: New orientation (90° roll) is not a duplicate
        new_orientation_1 = [math.pi/2, 0.0, 0.0]  # 90° roll
        self.assertFalse(m.is_duplicate_orientation(new_orientation_1))

        # Test 2: Slightly varied orientation (within tolerance) is a duplicate
        new_orientation_2 = [0.1, 0.0, 0.0]  # Within default tolerance (π/4)
        self.assertTrue(m.is_duplicate_orientation(new_orientation_2))

        # Test 3: Different orientation (90° yaw) is not a duplicate
        new_orientation_3 = [0.0, 0.0, math.pi/2]  # 90° yaw
        self.assertFalse(m.is_duplicate_orientation(new_orientation_3))

    def test_is_positioned_for_gate(self):
        """Test if the submarine is correctly positioned for gate approach."""
        m = GateMission()
        m.submarine_pose = self._make_pose(x=8.0, y=0.0, z=9.5)  # 2m in front of gate (x=10, z=10)

        # Case 1: Gate detected, submarine is within range (1.5m–2.5m distance)
        gate_near = _Obj(x=10.0, y=0.0, z=10.0)
        m.search_mission_object = lambda name: gate_near if name == "gate" else None
        self.assertTrue(m.is_positioned_for_gate())

        # Case 2: Submarine too far (3m away)
        gate_far = _Obj(x=13.0, y=0.0, z=10.0)
        m.search_mission_object = lambda name: gate_far if name == "gate" else None
        self.assertFalse(m.is_positioned_for_gate())

        # Case 3: Submarine missing pose or gate
        m.submarine_pose = None
        self.assertFalse(m.is_positioned_for_gate())
        m.submarine_pose = self._make_pose(x=8.0, y=0.0, z=9.5)
        m.search_mission_object = lambda name: None
        self.assertFalse(m.is_positioned_for_gate())

    def test_repeated_style_maneuvers(self):
        m = GateMission()
        m.previous_orientations = [[0.0, 0.0, 0.0]]

        # Try adding the same orientation repeatedly (should fail)
        for _ in range(5):
            self.assertTrue(m.is_duplicate_orientation([0.1, 0.0, 0.0]))  # Within tolerance
            self.assertEqual(len(m.previous_orientations), 1)  # No duplicates added

    def test_flickering_gate_detection(self):
        m = GateMission()
        m.controller_client = self._stub_action_client()
        m._REQUIRED_FRAMES = 2
        
        returns = [None, GATE]
        current = 0
        
        def flickering_search(name):
            nonlocal current
            if name == "gate":
                result = returns[current]
                current = (current + 1) % 2
                return result
            return None
       
        m.search_mission_object = flickering_search
        
        for _ in range(10):
            m.update_phase()

        self.assertEqual(m.current_phase, "Identify Images")
        self.assertFalse(m.gate_detected)

if __name__ == "__main__":
    import rostest
    rostest.rosrun("autonomy", "gate_mission_test", TestGateMissionScenarios)
