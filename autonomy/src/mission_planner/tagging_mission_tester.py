#!/usr/bin/env python3
import math
import time
from typing import Optional

import rospy
from geometry_msgs.msg import Point, PoseStamped

from tagging_mission import TaggingMission
from mission_planner.types import MissionObject


class ManualTaggingController:
    def __init__(self):
        rospy.init_node("manual_tagging_tester")

        # Initialize mission first
        self.mission = TaggingMission()
        self.mission.controller_client = self.create_mock_controller_client()
        self.mission.active = True

        # Mission parameters
        self.params = {
            "board_position": Point(x=8.0, y=0.0, z=-1.0),
            "board_confidence": 1.0,
            "target_animal": "reef_shark",
            "animal_confidence": 1.0,
            "submarine_speed": 0.8,
            "far_distance": 3.0,  # Distance for bonus points
        }

        # Calculate mission timing
        distance_to_board = math.sqrt(
            self.params["board_position"].x ** 2 +
            self.params["board_position"].y ** 2 +
            self.params["board_position"].z ** 2
        )
        self.time_to_approach = distance_to_board / self.params["submarine_speed"]

        # Set the target animal from previous mission
        self.mission.set_target_animal(self.params["target_animal"])

        # Add phase tracking
        self.phase_complete = {
            "search": False,
            "detect_config": False,
            "detect_distance": False,
            "approach": False,
            "fire_first": False,
            "fire_second": False,
        }

        # Publishers
        self.pose_pub = rospy.Publisher("/submarine_pose", PoseStamped, queue_size=10)

        # Timing and tracking
        self.start_time = time.time()
        self.torpedoes_fired = 0
        self.mission_complete = False

        # Mock torpedo service
        self.setup_torpedo_service()

    def create_mock_controller_client(self):
        """Create a properly mocked controller client"""
        client = type("MockControllerClient", (), {})()

        def send_goal(*args, **kwargs):
            return type("", (), {"status": "ACTIVE"})()

        def wait_for_result(timeout=None):
            return True

        def get_result():
            result = type("", (), {})()
            result.success = True
            return result

        client.send_goal = send_goal
        client.wait_for_result = wait_for_result
        client.get_result = get_result
        return client

    def setup_torpedo_service(self):
        """Set up torpedo firing service mock"""
        def mock_torpedo_service(torpedo_num):
            self.torpedoes_fired += 1
            result = type("", (), {})()
            
            if torpedo_num == 1:
                result.success = True
                result.message = "Torpedo 1 fired - full hit through opening"
                rospy.loginfo("Torpedo 1: Full hit!")
            elif torpedo_num == 2:
                result.success = True
                result.message = "Torpedo 2 fired - partial hit on board"
                rospy.loginfo("Torpedo 2: Partial hit!")
            else:
                result.success = False
                result.message = "No more torpedoes available"
                
            return result

        self.mission.fire_torpedo_service = mock_torpedo_service

    def get_object(self, name):
        """Simulate detected objects based on mission progress"""
        t = time.time() - self.start_time

        # Board appears after initial search
        if name == "board" and t > 1.0:
            return MissionObject(
                self.params["board_position"], "board", self.params["board_confidence"]
            )

        # Animals appear after board detection
        if name == "reef_shark" and t > 2.0:
            pos = Point()
            pos.x = self.params["board_position"].x
            pos.y = self.params["board_position"].y + 0.1  # Slightly right
            pos.z = self.params["board_position"].z + 0.3  # Above center
            return MissionObject(pos, "reef_shark", self.params["animal_confidence"])

        if name == "sawfish" and t > 2.0:
            pos = Point()
            pos.x = self.params["board_position"].x
            pos.y = self.params["board_position"].y - 0.1  # Slightly left
            pos.z = self.params["board_position"].z - 0.3  # Below center
            return MissionObject(pos, "sawfish", self.params["animal_confidence"])

        # Openings appear after animals are identified
        if name == "opening_1" and t > 3.0:
            pos = Point()
            pos.x = self.params["board_position"].x
            pos.y = self.params["board_position"].y + 0.15  # Near reef shark
            pos.z = self.params["board_position"].z + 0.25
            return MissionObject(pos, "opening_1", 1.0)

        if name == "opening_2" and t > 3.0:
            pos = Point()
            pos.x = self.params["board_position"].x
            pos.y = self.params["board_position"].y - 0.15  # Near sawfish
            pos.z = self.params["board_position"].z - 0.25
            return MissionObject(pos, "opening_2", 1.0)

        # Horizontal bars for far distance detection
        if name == "horizontal_bar_1" and t > 1.5:
            pos = Point()
            pos.x = self.params["board_position"].x - self.params["far_distance"]
            pos.y = -1.0
            pos.z = -1.0
            return MissionObject(pos, "horizontal_bar_1", 1.0)

        if name == "horizontal_bar_2" and t > 1.5:
            pos = Point()
            pos.x = self.params["board_position"].x - self.params["far_distance"]
            pos.y = 1.0
            pos.z = -1.0
            return MissionObject(pos, "horizontal_bar_2", 1.0)

        return None

    def get_submarine_pose(self):
        """Simulate submarine movement through the mission"""
        t = time.time() - self.start_time
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"

        # Phase timing
        search_time = 2.0
        config_time = 4.0
        distance_time = 6.0
        approach_time = self.time_to_approach + 8.0
        fire_first_time = approach_time + 2.0
        fire_second_time = fire_first_time + 2.0

        # Update phases based on progress
        if t < search_time:
            self.mission.current_phase = "Search for Board"
            self.phase_complete["search"] = False
            # Start position
            p.pose.position.x = 0.0
            p.pose.position.y = 0.0
            p.pose.position.z = -0.5
        elif t < config_time:
            self.mission.current_phase = "Detect Board Configuration"
            self.phase_complete["search"] = True
            self.phase_complete["detect_config"] = False
            # Slow approach for detection
            progress = (t - search_time) / (config_time - search_time)
            p.pose.position.x = self.params["board_position"].x * 0.3 * progress
            p.pose.position.y = 0.0
            p.pose.position.z = -0.5
        elif t < distance_time:
            self.mission.current_phase = "Detect Far Distance"
            self.phase_complete["detect_config"] = True
            self.phase_complete["detect_distance"] = False
            # Continue slow approach
            p.pose.position.x = self.params["board_position"].x * 0.4
            p.pose.position.y = 0.0
            p.pose.position.z = -0.5
        elif t < approach_time:
            self.mission.current_phase = "Approach Board"
            self.phase_complete["detect_distance"] = True
            self.phase_complete["approach"] = False
            # Move to far distance position for firing
            progress = (t - distance_time) / (approach_time - distance_time)
            target_x = self.params["board_position"].x - self.params["far_distance"]
            start_x = self.params["board_position"].x * 0.4
            p.pose.position.x = start_x + (target_x - start_x) * progress
            p.pose.position.y = 0.0
            p.pose.position.z = -0.5
        elif t < fire_first_time:
            self.mission.current_phase = "Fire First Torpedo"
            self.phase_complete["approach"] = True
            self.phase_complete["fire_first"] = False
            # Hold position at far distance
            p.pose.position.x = self.params["board_position"].x - self.params["far_distance"]
            p.pose.position.y = 0.0
            p.pose.position.z = -0.5
        elif t < fire_second_time:
            self.mission.current_phase = "Fire Second Torpedo"
            self.phase_complete["fire_first"] = True
            self.phase_complete["fire_second"] = False
            # Maintain firing position
            p.pose.position.x = self.params["board_position"].x - self.params["far_distance"]
            p.pose.position.y = 0.0
            p.pose.position.z = -0.5
        else:
            self.mission.current_phase = "Mission Complete"
            self.phase_complete["fire_second"] = True
            self.mission_complete = True
            # Move away and surface
            p.pose.position.x = self.params["board_position"].x - self.params["far_distance"]
            p.pose.position.y = 0.0
            p.pose.position.z = 0.0  # Surface

        # Add slight realistic movement
        p.pose.position.x += 0.05 * math.sin(t * 0.5)
        p.pose.position.y += 0.03 * math.cos(t * 0.7)
        p.pose.position.z += 0.02 * math.sin(t * 0.3)

        return p

    def run(self):
        """Run the tagging mission test"""
        rate = rospy.Rate(10)
        last_phase = None

        while not rospy.is_shutdown() and not self.mission_complete:
            t = time.time() - self.start_time

            # Update mission state
            pose = self.get_submarine_pose()
            self.mission.submarine_pose = pose
            self.pose_pub.publish(pose)

            # Update mission
            self.mission.search_mission_object = self.get_object
            self.update_mission_phases(t)  # Add manual phase progression
            self.mission.run()

            # Print phase transitions
            if self.mission.current_phase != last_phase:
                rospy.loginfo(f"\n=== Phase: {self.mission.current_phase} ===")
                last_phase = self.mission.current_phase

            # Status updates every second
            if rospy.get_time() % 1.0 < 0.1:
                completion = sum(1 for p in self.phase_complete.values() if p)
                distance_to_board = math.sqrt(
                    (pose.pose.position.x - self.params["board_position"].x) ** 2 +
                    (pose.pose.position.y - self.params["board_position"].y) ** 2 +
                    (pose.pose.position.z - self.params["board_position"].z) ** 2
                )

                rospy.loginfo(
                    f"Time: {t:.1f}s | "
                    f"Phase: {self.mission.current_phase} ({completion}/6 complete) | "
                    f"Position: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, {pose.pose.position.z:.2f}) | "
                    f"Distance to board: {distance_to_board:.2f}m | "
                    f"Target: {self.mission.target_animal} | "
                    f"Torpedoes: {self.torpedoes_fired}/2 | "
                    f"Score: {self.mission.score['total']}"
                )

            rate.sleep()

        # Final results
        self.mission.calculate_final_score()
        
        rospy.loginfo("\n=== TAGGING MISSION COMPLETE ===")
        rospy.loginfo(f"Result: {'SUCCESS' if self.mission.completed else 'INCOMPLETE'}")
        rospy.loginfo(f"Target Animal: {self.mission.target_animal}")
        rospy.loginfo(f"Board Configuration: {self.mission.board_configuration}")
        rospy.loginfo(f"Torpedoes Fired: {self.torpedoes_fired}/2")
        rospy.loginfo(f"Far Distance: {self.mission.far_distance:.2f}m")
        rospy.loginfo(f"Duration: {time.time() - self.start_time:.1f}s")
        rospy.loginfo(f"Final Score: {self.mission.score['total']}")
        rospy.loginfo("\n--- SCORING BREAKDOWN ---")
        for key, value in self.mission.score.items():
            if value > 0:
                rospy.loginfo(f"{key.replace('_', ' ').title()}: {value}")
        rospy.loginfo("="*50)
        
        rospy.signal_shutdown("Tagging mission complete")

    def update_mission_phases(self, t):
        """Manually progress mission phases and trigger actions"""
        # Board detection
        if t > 2 and not self.phase_complete["search"]:
            self.phase_complete["search"] = True
            rospy.loginfo("Board detected!")
            
        # Configuration detection
        if t > 4 and not self.phase_complete["detect_config"]:
            self.mission.detect_board_configuration()
            self.phase_complete["detect_config"] = True
            rospy.loginfo(f"Board configuration: {self.mission.board_configuration}")
            
        # Distance detection  
        if t > 6 and not self.phase_complete["detect_distance"]:
            self.mission.detect_far_distance()
            self.phase_complete["detect_distance"] = True
            rospy.loginfo(f"Far distance: {self.mission.far_distance}")
            
        # Openings detection
        if t > 8 and self.phase_complete["detect_distance"]:
            self.mission.detect_openings()
            rospy.loginfo("Openings detected!")
            
        # Fire first torpedo
        if t > 12 and not self.phase_complete["fire_first"]:
            self.mission.fire_torpedo()
            self.phase_complete["fire_first"] = True
            
        # Fire second torpedo  
        if t > 16 and not self.phase_complete["fire_second"] and self.phase_complete["fire_first"]:
            self.mission.fire_torpedo()
            self.phase_complete["fire_second"] = True


if __name__ == "__main__":
    tester = ManualTaggingController()
    try:
        tester.run()
    except rospy.ROSInterruptException:
        pass