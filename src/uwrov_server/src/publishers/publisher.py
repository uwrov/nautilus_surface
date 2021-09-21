# Base class for any publishers to be added to the server
import rospy
from abc import ABC, abstractmethod


class ServerPub(ABC):
    """
    Base class for any publishers added to the OceanUI server.
    Transfers data from OceanUI to a ROS topic.
    """
    @abstractmethod
    def __init__(self, topic: str, type: any, queue_size: int = 1):
        self.publisher = rospy.Publisher(topic, type, queue_size=queue_size)

    @abstractmethod
    def publish(self, msg):
        pass
