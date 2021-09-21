# Base class for any publishers to be added to the server
import rospy
from abc import ABC, abstractmethod

# Take in data from the interface and publishes to a ROS topic


class ServerPub(ABC):
    @abstractmethod
    def __init__(self, topic: str, type: any, queue_size: int = 1):
        self.publisher = rospy.Publisher(topic, type, queue_size=queue_size)

    @abstractmethod
    def publish(self, msg):
        pass
