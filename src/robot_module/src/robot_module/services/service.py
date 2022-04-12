# Base class for services available for the robot module
# Although as of now not entirely necessary, any function
# that needs to be uniform across services should be updated
# here in the future.
from abc import ABC, abstractmethod

class Service(ABC):
    """
    Base class for any subscribers added to the OceanUI server.
    Transfers data from ROS topics to OceanUI.
    """
    @abstractmethod
    def __init__(self, node_name):
        self.node_name = node_name

    @abstractmethod
    def setup(self, *args):
        raise NotImplementedError
