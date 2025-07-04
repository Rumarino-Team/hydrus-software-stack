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
