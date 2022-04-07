#!/usr/bin/env python3
import rospy

from .publisher import ServerPub
from std_msgs.msg import String

class LockPub(ServerPub):
    """Publisher for locking command priority to a specific RobotModual."""

    def __init__(self, topic):
        super().__init__(topic, String, queue_size=10)

    def request_priority(self, node_name):
        self.publisher.publish(node_name)