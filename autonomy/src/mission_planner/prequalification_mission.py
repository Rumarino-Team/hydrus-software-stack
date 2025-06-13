#!/usr/bin/env python3
# mission_planner/prequalification_mission.py
import math
from typing import Callable, List, Optional, Set

import actionlib
import rospy
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String

from autonomy.msg import (
    Detection,
    Detections,
    NavigateToWaypointAction,
    NavigateToWaypointFeedback,
    NavigateToWaypointGoal,
    NavigateToWaypointResult,
)
from mission_planner.base_mission import BaseMission
from mission_planner.mission_tree import MissionTreeNode
from mission_planner.task_parameters import TaskParameters

# Change to absolute imports
from mission_planner.types import MissionObject, TaskType


class PreQualificationMission(BaseMission):
    def __init__(self):
        # Initialize the base mission
        super().__init__(name="Pre-Qualification")

        # Configuration specific to Pre-Qualification
        self.cls_names = {1: "Gate", 2: "Buoy"}

        # Additional publisher for status
        self.status_pub = rospy.Publisher("/mission_status", String, queue_size=10)

        # Build the mission tree
        self.mission_tree_root = self.build_mission_tree()

    def build_mission_tree(self) -> MissionTreeNode:
        root = MissionTreeNode(
            task=TaskType.HOLD_POSITION,
            name="Mission Start",
            action=lambda: True,  # Grouping node
        )

        # Gate Branch
        search_gate_node = MissionTreeNode(
            task=TaskType.SEARCH,
            object_cls="Gate",
            name="Search for Gate",
            condition=lambda: "Gate" not in self.detected_objects,
            action=self.execute_search,
        )
        approach_gate_node = MissionTreeNode(
            task=TaskType.MOVE_TO_CENTER,
            object_cls="Gate",
            name="Approach Gate",
            condition=lambda: "Gate" in self.detected_objects,
            action=self.create_action(TaskType.MOVE_TO_CENTER, "Gate"),
        )
        gate_branch_node = MissionTreeNode(
            task=TaskType.HOLD_POSITION,
            name="Gate Branch",
            action=lambda: True,  # Grouping node
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
            action=self.create_action(TaskType.MOVE_AROUND, "Buoy"),
        )
        root.add_child(buoy_node)

        # Re-approach Gate
        reapproach_gate_node = MissionTreeNode(
            task=TaskType.MOVE_TO_CENTER,
            object_cls="Gate",
            name="Re-approach Gate",
            condition=lambda: "Gate" in self.detected_objects,
            action=self.create_action(TaskType.MOVE_TO_CENTER, "Gate"),
        )
        root.add_child(reapproach_gate_node)

        # Surface
        surface_node = MissionTreeNode(
            task=TaskType.SURFACE,
            name="Surface",
            condition=lambda: True,
            action=self.create_action(TaskType.SURFACE, None),
        )
        root.add_child(surface_node)

        return root

    def get_status(self) -> dict:
        """Get the current status of the prequalification mission"""
        return {
            "name": self.name,
            "objects_detected": list(self.detected_objects),
            "completion_percentage": (
                0
                if not self.mission_tree_root
                else (
                    100
                    if self.mission_tree_root.completed
                    else self.mission_tree_root.get_progress() * 100
                )
            ),
        }

    def run(self):
        """Run one iteration of the mission"""
        if not self.submarine_pose:
            rospy.logwarn_throttle(1, "Waiting for submarine pose...")
            return

        self.update_detected_objects()

        # Publish status
        status_msg = String()
        status_dict = self.get_status()
        status_msg.data = (
            f"{self.name} Mission: Progress {status_dict['completion_percentage']:.1f}%, "
            f"Detected: {', '.join(status_dict['objects_detected'])}"
        )
        self.status_pub.publish(status_msg)

        if (
            self.mission_tree_root
            and not self.mission_tree_root.completed
            and self.active
        ):
            self.mission_tree_root.execute()
            if self.mission_tree_root.completed:
                self.completed = True
                rospy.loginfo(f"Mission '{self.name}' completed!")
