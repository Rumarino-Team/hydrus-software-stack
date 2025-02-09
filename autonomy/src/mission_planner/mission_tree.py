# mission_planner/mission_tree.py
import rospy
from dataclasses import dataclass, field
from typing import List, Optional, Callable
from mission_planner.types import TaskType

@dataclass
class MissionTreeNode:
    task: TaskType
    object_cls: Optional[str] = None
    condition: Callable[[], bool] = lambda: True
    action: Optional[Callable[[], bool]] = None
    children: List["MissionTreeNode"] = field(default_factory=list)
    name: str = "Unnamed Node"
    completed: bool = False

    def add_child(self, child: "MissionTreeNode"):
        self.children.append(child)

    def execute(self) -> bool:
        if self.completed:
            rospy.loginfo(f"Node '{self.name}' already completed. Skipping execution.")
            return True

        if not self.condition():
            rospy.loginfo(f"Condition for '{self.name}' not met, skipping node.")
            return False

        if self.action:
            rospy.loginfo(f"Executing node '{self.name}' with task {self.task.name}")
            success = self.action()
            if success:
                rospy.loginfo(f"Node '{self.name}' completed successfully.")
                self.completed = True
                # Execute children sequentially
                for child in self.children:
                    child.execute()
                return True
            else:
                rospy.loginfo(f"Node '{self.name}' failed. Trying alternate branches if any...")
                for child in self.children:
                    if child.execute():
                        return True
                return False
        else:
            rospy.loginfo(f"Node '{self.name}' has no direct action. Executing children nodes.")
            for child in self.children:
                child.execute()
            self.completed = True
            return True
