#!/usr/bin/env python3
# mission_planner/mission_manager.py
import rospy
import json
from typing import Dict, List, Optional, Type
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse

# Change to absolute imports instead of relative imports
from mission_planner.base_mission import BaseMission
from mission_planner.prequalification_mission import PreQualificationMission
from mission_planner.slalom_mission import SlalomMission
from mission_planner.tagging_mission import TaggingMission
from mission_planner.gate_mission import GateMission

class MissionManager:
    """    roslaunch autonomy mission_planner.launch
    Manages multiple missions and coordinates their execution.
    Provides services to start/stop/select missions and publishes status information.
    """
    
    def __init__(self):
        rospy.init_node('mission_manager', anonymous=False)
        rospy.loginfo("Initializing Mission Manager...")
        
        # Available missions
        self.missions: Dict[str, BaseMission] = {}
        self.active_mission: Optional[BaseMission] = None
        self.mission_status: Dict[str, dict] = {}
        
        # Register missions
        self.register_mission("prequalification", PreQualificationMission())
        self.register_mission("slalom", SlalomMission())
        self.register_mission("gate", GateMission())
        self.register_mission("tagging", TaggingMission())
        
        # Set default mission
        self.select_mission("prequalification")
        
        # Mission dependencies - to share information between missions
        self.mission_dependencies = {
            "tagging": ["gate"]  # Tagging mission depends on Gate mission
        }
        
        # Publishers
        self.status_pub = rospy.Publisher('/mission_manager/status', String, queue_size=10)
        
        # Services
        rospy.Service('/mission_manager/start', Trigger, self.handle_start)
        rospy.Service('/mission_manager/stop', Trigger, self.handle_stop)
        rospy.Service('/mission_manager/reset', Trigger, self.handle_reset)
        rospy.Service('/mission_manager/select_mission', SetBool, self.handle_select_mission)
        
        # Status update timer
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)
        
        rospy.loginfo("Mission Manager initialized. Available missions: " + 
                     ", ".join(self.missions.keys()))
    
    def register_mission(self, name: str, mission: BaseMission):
        """Register a mission with the manager"""
        self.missions[name] = mission
        rospy.loginfo(f"Registered mission: {name}")
    
    def select_mission(self, mission_name: str) -> bool:
        """Select a mission by name"""
        if mission_name not in self.missions:
            rospy.logwarn(f"Mission '{mission_name}' not found")
            return False
        
        # Deactivate current mission if any
        if self.active_mission:
            self.active_mission.deactivate()
        
        # Set the new active mission
        self.active_mission = self.missions[mission_name]
        rospy.loginfo(f"Selected mission: {mission_name}")
        
        # Apply mission dependencies if any
        self.apply_mission_dependencies(mission_name)
        
        return True

    def apply_mission_dependencies(self, mission_name: str):
        """Apply dependencies for the selected mission"""
        if mission_name in self.mission_dependencies:
            for dependency in self.mission_dependencies[mission_name]:
                if dependency in self.missions:
                    # Check if the dependent mission is completed
                    dependency_mission = self.missions[dependency]
                    
                    if dependency == "gate" and mission_name == "tagging":
                        # Gate mission dependency for Tagging mission
                        gate_mission = self.missions["gate"]
                        tagging_mission = self.missions["tagging"]
                        
                        # If gate mission is completed, get the chosen animal
                        if gate_mission.completed:
                            chosen_animal = gate_mission.get_chosen_animal()
                            tagging_mission.set_target_animal(chosen_animal)
                            rospy.loginfo(f"Applied dependency: Tagging mission will target {chosen_animal}")
                        else:
                            rospy.logwarn(f"Dependency '{dependency}' for mission '{mission_name}' is not completed")
                else:
                    rospy.logwarn(f"Dependency '{dependency}' for mission '{mission_name}' not found")
    
    def start_mission(self) -> bool:
        """Start the active mission"""
        if not self.active_mission:
            rospy.logwarn("No active mission to start")
            return False
        
        self.active_mission.activate()
        rospy.loginfo(f"Started mission: {self.active_mission.name}")
        return True
    
    def stop_mission(self) -> bool:
        """Stop the active mission"""
        if not self.active_mission:
            rospy.logwarn("No active mission to stop")
            return False
        
        self.active_mission.deactivate()
        rospy.loginfo(f"Stopped mission: {self.active_mission.name}")
        return True
    
    def reset_mission(self) -> bool:
        """Reset the active mission"""
        if not self.active_mission:
            rospy.logwarn("No active mission to reset")
            return False
        
        self.active_mission.reset()
        rospy.loginfo(f"Reset mission: {self.active_mission.name}")
        return True
    
    def update_status(self):
        """Update status information for all missions"""
        for name, mission in self.missions.items():
            self.mission_status[name] = mission.get_status()
        
        # Publish overall status
        status_msg = String()
        status_dict = {
            "active_mission": self.active_mission.name if self.active_mission else "None",
            "missions": self.mission_status
        }
        status_msg.data = json.dumps(status_dict)
        self.status_pub.publish(status_msg)
    
    # Service handlers
    def handle_start(self, req):
        """Handle start service request"""
        success = self.start_mission()
        return TriggerResponse(
            success=success,
            message=f"Started mission: {self.active_mission.name}" if success else "Failed to start mission"
        )
    
    def handle_stop(self, req):
        """Handle stop service request"""
        success = self.stop_mission()
        return TriggerResponse(
            success=success,
            message=f"Stopped mission: {self.active_mission.name}" if success else "Failed to stop mission"
        )
    
    def handle_reset(self, req):
        """Handle reset service request"""
        success = self.reset_mission()
        return TriggerResponse(
            success=success,
            message=f"Reset mission: {self.active_mission.name}" if success else "Failed to reset mission"
        )
    
    def handle_select_mission(self, req):
        """Handle select mission service request"""
        mission_name = req.data
        success = self.select_mission(mission_name)
        return SetBoolResponse(
            success=success,
            message=f"Selected mission: {mission_name}" if success else f"Failed to select mission: {mission_name}"
        )
    
    # Timer callback
    def timer_callback(self, event):
        """Timer callback to update status and run mission iterations"""
        # Update status information
        self.update_status()
        
        # Run active mission if any
        if self.active_mission and self.active_mission.active:
            self.active_mission.run()
    
    def run(self):
        """Main run loop"""
        rospy.spin()

if __name__ == "__main__":
    try:
        manager = MissionManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass