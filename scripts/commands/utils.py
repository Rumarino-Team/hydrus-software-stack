from abc import ABC, abstractmethod


class Command(ABC):
    """
    Abstract base class for commands.
    All commands should inherit from this class and implement the `execute` method.
    """

    def __init__(self, name, description):
        """
        Initialize the command with a name and description.

        :param name: The name of the command.
        :param description: A brief description of what the command does.
        """
        self.name = name
        self.description = description

    @abstractmethod
    def execute(self, *args, **kwargs):
        """
        Execute the command with the given arguments.
        """
        pass


from pathlib import Path


def get_building_path(volume: bool = False) -> Path:
    """Get the path where packages are built. Inside the copy ROS workspace or the volume."""
    # Determine ROS directory based on volume usage
    if volume:
        print("Using Volume directory for tmux scripts: /home/catkin_ws")
        return Path("/home/catkin_ws")
    else:
        print("Using Docker container directory: /catkin_ws")
        return Path("/catkin_ws")
