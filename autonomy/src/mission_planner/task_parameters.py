#!/usr/bin/env python3
# mission_planner/task_parameters.py

class TaskParameters:
    """
    Configuration parameters for different task types
    """
    
    def __init__(self):
        # Hold position parameters
        self.hold_time = 5.0  # seconds
        
        # Move around parameters
        self.radius = 2.0  # meters
        self.circle_points = 8  # number of points to use in circle
        
        # Search parameters
        self.search_timeout = 60.0  # seconds
        self.search_speed = 0.2  # m/s
        
        # Navigation parameters
        self.approach_distance = 1.5  # meters
        self.success_threshold = 0.5  # meters