import rospy
from .publisher import ServerPub
from std_msgs.msg import Int16


class ChannelPub(ServerPub):
    """
    Publisher for switching the viewed camera channel
    """

    def __init__(self, topic):
        super().__init__(topic, Int16)
        self.msg = Int16()

    def publish(self, channel):
        rospy.loginfo("Changing channel to %d", channel)
        self.msg.data = channel
        self.publisher.publish(self.msg)
