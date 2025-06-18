#!/usr/bin/env python3
"""
Fixed Manual Gate Mission Tester
"""
import math
import time
from typing import Optional

import rospy
from geometry_msgs.msg import Point, PoseStamped

from mission_planner.gate_mission import GateMission
from mission_planner.types import MissionObject


class ManualMissionController:
    def __init__(self):
        rospy.init_node("manual_gate_tester")

        # Initialize mission first to get the default animal
        self.mission = GateMission()
        self.mission.controller_client = self.create_mock_controller_client()
        self.mission.active = True

        # Now match parameters with gate_mission.py defaults
        self.params = {
            "gate_position": Point(x=10.0, y=3.0, z=10.0),
            "gate_confidence": 1.0,
            "animal_type": self.mission.chosen_animal,  # Get default from GateMission
            "animal_position": Point(x=10.0, y=3.0, z=10.0),
            "animal_confidence": 1.0,
            "submarine_speed": 0.9,
            "style_points": 0,  # Let gate_mission.py handle points
        }

        # Adjust total distance based on new gate position
        distance_to_gate = math.sqrt(
            self.params["gate_position"].x ** 2
            + self.params["gate_position"].y ** 2
            + self.params["gate_position"].z ** 2
        )
        distance_total = distance_to_gate + 3.0  # Add 3m for passing through gate
        min_duration = (
            distance_total / self.params["submarine_speed"]
        ) + 10.0  # More buffer time
        self.params["mission_duration"] = min_duration

        # Add phase tracking
        self.phase_complete = {
            "search": False,
            "identify": False,
            "choose": False,
            "approach": False,
            "navigate": False,
        }

        # Fixed controller client implementation
        self.mission = GateMission()
        self.mission.controller_client = self.create_mock_controller_client()
        self.mission.active = True
        self.mission.chosen_animal = self.params["animal_type"]  # Set explicitly

        # Publishers
        self.pose_pub = rospy.Publisher("/submarine_pose", PoseStamped, queue_size=10)
        self.obj_pub = rospy.Publisher("/detected_objects", Point, queue_size=10)

        # Timing
        self.start_time = time.time()
        self.gate_passed = False
        self.style_awarded = False

        # Add orientation tracking
        self.last_orientation = None
        self.visited_orientations = set()  # Track orientations we've been in
        self.style_points = 0

        # Define point values for different rotations
        self.rotation_points = {
            "roll": 2,  # Worth more points
            "pitch": 2,  # Worth more points
            "yaw": 1,  # Worth less points
        }

    def create_mock_controller_client(self):
        """Create a properly mocked controller client"""
        client = type("MockControllerClient", (), {})()

        def send_goal(*args, **kwargs):
            return type("", (), {"status": "ACTIVE"})()

        def wait_for_result(timeout=None):  # Now accepts timeout parameter
            return True

        def get_result():
            result = type("", (), {})()
            result.success = True
            return result

        client.send_goal = send_goal
        client.wait_for_result = wait_for_result
        client.get_result = get_result
        return client

    def get_object(self, name):
        t = time.time() - self.start_time

        # Always return gate and divider after initial delay
        if name == "gate" and t > 0.5:
            return MissionObject(
                self.params["gate_position"], "gate", self.params["gate_confidence"]
            )

        if name == "gate_divider" and t > 1.0:
            pos = Point()
            pos.x = self.params["gate_position"].x
            pos.y = self.params["gate_position"].y
            pos.z = self.params["gate_position"].z
            return MissionObject(pos, "gate_divider", 1.0)

        # Return both animals with fixed offsets
        if name in ["reef_shark", "sawfish"] and t > 2.0:
            pos = Point()
            pos.x = self.params["gate_position"].x
            pos.y = self.params["gate_position"].y + (
                0.5 if name == "reef_shark" else -0.5
            )
            pos.z = self.params["gate_position"].z
            return MissionObject(pos, name, self.params["animal_confidence"])

        return None

    def get_submarine_pose(self):
        t = time.time() - self.start_time
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"

        # Calculate time needed to reach gate based on new distance
        distance_to_gate = math.sqrt(
            self.params["gate_position"].x ** 2
            + self.params["gate_position"].y ** 2
            + self.params["gate_position"].z ** 2
        )
        time_to_gate = distance_to_gate / self.params["submarine_speed"]

        # Phase timing control
        search_time = 2.0  # Time to complete search
        identify_time = 4.0  # Time to identify images
        choose_time = 5.0  # Time to choose side
        approach_time = time_to_gate  # Time to reach gate

        # Update phases based on progress
        if t < search_time:
            self.mission.current_phase = "Search for Gate"
            self.phase_complete["search"] = False
        elif t < identify_time:
            self.mission.current_phase = "Identify Images"
            self.phase_complete["search"] = True
            self.phase_complete["identify"] = False
        elif t < choose_time:
            self.mission.current_phase = "Choose Side"
            self.phase_complete["identify"] = True
            self.phase_complete["choose"] = False
        elif t < approach_time:
            self.mission.current_phase = "Approach Gate"
            self.phase_complete["choose"] = True
            self.phase_complete["approach"] = False
            # Linear interpolation from start (0,0,0) to gate position
            progress = t / time_to_gate
            p.pose.position.x = self.params["gate_position"].x * progress
            p.pose.position.y = self.params["gate_position"].y * progress
            p.pose.position.z = self.params["gate_position"].z * progress
        elif t < (time_to_gate + 3.0):  # Pass through
            self.mission.current_phase = "Navigate Gate with Style"
            self.phase_complete["approach"] = True
            # Use submarine_speed for passing through
            delta_x = self.params["submarine_speed"] * (t - time_to_gate)
            p.pose.position.x = self.params["gate_position"].x + delta_x
            p.pose.position.y = self.params["gate_position"].y
            p.pose.position.z = self.params["gate_position"].z

            if not self.gate_passed and delta_x > 1.0:  # Pass gate after 1m through
                self.gate_passed = True
                self.phase_complete["navigate"] = True
                rospy.loginfo("Gate passed successfully!")
                self.mission.gate_passed = True
                self.mission.style_completed = True
        else:  # Surface after passing
            self.mission.current_phase = "Surface"
            p.pose.position.x = self.params["gate_position"].x + 3.0
            p.pose.position.z += 0.5  # Move upward

        p.pose.position.y += 0.1 * math.sin(t)
        p.pose.position.z += -0.1 * math.sin(0.5 * t)

        # Add orientation changes during gate passage
        if time_to_gate < t < (time_to_gate + 3.0):
            # Calculate orientations (in degrees)
            roll = math.degrees(math.sin(t * math.pi)) * 90  # Full 90° roll
            pitch = math.degrees(math.sin(t * math.pi / 2)) * 90  # Full 90° pitch
            yaw = math.degrees(math.sin(t * math.pi / 3)) * 90  # Full 90° yaw

            # Convert to quaternion and set pose orientation
            # ... (quaternion conversion code here)

            # Check for 90° changes and award points
            current_orientation = (int(roll / 90), int(pitch / 90), int(yaw / 90))
            if self.last_orientation is not None:
                # Check which axis changed by 90°
                if current_orientation not in self.visited_orientations:
                    roll_change = abs(current_orientation[0] - self.last_orientation[0])
                    pitch_change = abs(
                        current_orientation[1] - self.last_orientation[1]
                    )
                    yaw_change = abs(current_orientation[2] - self.last_orientation[2])

                    if roll_change == 1:
                        self.style_points += self.rotation_points["roll"]
                        rospy.loginfo("Added 2 points for roll")
                    if pitch_change == 1:
                        self.style_points += self.rotation_points["pitch"]
                        rospy.loginfo("Added 2 points for pitch")
                    if yaw_change == 1:
                        self.style_points += self.rotation_points["yaw"]
                        rospy.loginfo("Added 1 point for yaw")

                    self.visited_orientations.add(current_orientation)
                    self.mission.style_points = self.style_points

            self.last_orientation = current_orientation

        return p

    def run(self):
        rate = rospy.Rate(10)
        last_phase = None

        while not rospy.is_shutdown() and not self.gate_passed:
            t = time.time() - self.start_time

            # Update mission state
            pose = self.get_submarine_pose()
            self.mission.submarine_pose = pose
            self.pose_pub.publish(pose)

            # Update mission
            self.mission.search_mission_object = self.get_object
            self.mission.run()

            # Print phase transitions and status
            if self.mission.current_phase != last_phase:
                rospy.loginfo(f"\n=== Phase: {self.mission.current_phase} ===")
                last_phase = self.mission.current_phase

            if rospy.get_time() % 1.0 < 0.1:
                status = self.mission.get_status()
                completion = sum(1 for p in self.phase_complete.values() if p)
                rospy.loginfo(
                    f"Time: {t:.1f}s | "
                    f"Phase: {self.mission.current_phase} ({completion}/5 complete) | "
                    f"Position: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, {pose.pose.position.z:.2f}) | "
                    f"Animal: {self.mission.chosen_animal} | "
                    f"Style: {status['style_points']}"
                )

            rate.sleep()

        # Final status with position
        status = self.mission.get_status()
        final_pose = self.get_submarine_pose()

        rospy.loginfo("\n=== MISSION COMPLETE ===")
        rospy.loginfo(f"Result: {'SUCCESS' if status['gate_passed'] else 'FAIL'}")
        rospy.loginfo(
            f"Final Position: ({final_pose.pose.position.x:.2f}, {final_pose.pose.position.y:.2f}, {final_pose.pose.position.z:.2f})"
        )
        rospy.loginfo(f"Animal: {self.mission.chosen_animal}")
        rospy.loginfo(f"Style Points: {status['style_points']}")
        rospy.loginfo(f"Duration: {time.time() - self.start_time:.1f}s")
        rospy.signal_shutdown("Mission complete")


if __name__ == "__main__":
    tester = ManualMissionController()
    try:
        tester.run()
    except rospy.ROSInterruptException:
        pass
