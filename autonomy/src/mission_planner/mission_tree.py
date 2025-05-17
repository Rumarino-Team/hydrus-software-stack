#!/usr/bin/env python3
# mission_planner/mission_tree.py
"""
Implementation of a mission tree structure for behavior-based mission execution.
"""

import rospy
from typing import List, Optional, Callable
from mission_planner.types import TaskType

class MissionTreeNode:
    """
    A node in a mission tree that represents a task or a group of tasks.
    The tree is executed in a depth-first manner, with each node executing 
    its action if its condition is met.
    """
    
    def __init__(
        self, 
        task: TaskType, 
        name: str, 
        object_cls: str = None,
        condition: Callable[[], bool] = lambda: True,
        action: Callable[[], bool] = None
    ):
        # Node metadata
        self.name = name
        self.task = task
        self.object_cls = object_cls
        
        # Execution attributes
        self.children: List[MissionTreeNode] = []
        self.condition = condition  # Function that returns True if this node can execute
        self.action = action        # Function that performs the node's action, returns True when complete
        self.completed = False      # Whether this node has completed its action
        self.started = False        # Whether this node has started execution
        self.current_child_index = 0
        
        # Start time for measuring progress
        self.start_time = None
        self.last_execution = None
        
    def add_child(self, child):
        """Add a child node to this node"""
        self.children.append(child)
        return self
        
    def execute(self) -> bool:
        """
        Execute this node and its children.
        Returns True if the node and all its children are completed.
        """
        # Skip if already completed
        if self.completed:
            return True
            
        # Check if condition is met
        if not self.condition():
            return False
            
        # Record start time
        if not self.started:
            self.started = True
            self.start_time = rospy.Time.now()
            rospy.loginfo(f"Starting node: {self.name}")
            
        # Update last execution time
        self.last_execution = rospy.Time.now()
        
        # If we have children, execute them in order
        if self.children:
            # Get the current child
            if self.current_child_index < len(self.children):
                child = self.children[self.current_child_index]
                
                # Execute the child
                if child.execute():
                    # Child is completed, move to next child
                    self.current_child_index += 1
                
                # All children completed?
                if self.current_child_index >= len(self.children):
                    self.completed = True
                    rospy.loginfo(f"Completed node: {self.name}")
                    return True
                    
                return False
                
        # No children or all children completed, execute our action
        elif self.action:
            result = self.action()
            if result:
                self.completed = True
                rospy.loginfo(f"Completed node: {self.name}")
                
            return result
            
        # No action to execute
        else:
            self.completed = True
            rospy.loginfo(f"Completed node: {self.name} (no action)")
            return True
            
        return False
        
    def reset(self):
        """Reset this node and all its children"""
        self.completed = False
        self.started = False
        self.current_child_index = 0
        
        for child in self.children:
            child.reset()
            
    def get_progress(self) -> float:
        """Get a value between 0 and 1 representing the progress of this node"""
        if not self.started:
            return 0.0
            
        if self.completed:
            return 1.0
            
        if not self.children:
            return 0.5 if self.started else 0.0
            
        # Calculate progress based on children
        child_progress_sum = 0.0
        for i, child in enumerate(self.children):
            if i < self.current_child_index:
                # Completed children contribute fully
                child_progress_sum += 1.0
            elif i == self.current_child_index:
                # Current child contributes partially
                child_progress_sum += child.get_progress()
                
        return child_progress_sum / len(self.children) if self.children else 0.0
