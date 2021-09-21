# Some base classes to write servers which bridge
# our interface and ROS topics
import rospy
from abc import ABC, abstractmethod


class ServerSub(ABC):
    """
    Base class for any subscribers added to the OceanUI server.
    Transfers data from ROS topics to OceanUI.
    """
    @abstractmethod
    def __init__(self, topic: str, type: any,
                 args: tuple = None, queue_size: int = 1):
        self.sub = rospy.Subscriber(
            topic, type, self.callback, args, queue_size=queue_size)

    @abstractmethod
    def callback(self, msg, args=None):
        raise NotImplementedError
