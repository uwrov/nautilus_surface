import rospy
from .subscriber import ServerSub

class LockSub(ServerSub):
    """Subscriber to lock topic"""

    def __init__(self, topic):
        """Init lock subscriber"""
        super().__init__(topic, String)
        self.node = ""

    def callback(self, priority_node):
        """Callback for lock subscriber"""
        self.node = priority_node.data
