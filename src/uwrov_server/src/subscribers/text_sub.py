import rospy
from std_msgs.msg import String

from .subscriber import SeverSub


class TextSub(SeverSub):
    def __init__(self, topic):
        super().__init__(topic, String)

    def callback(self, msg):
        rospy.loginfo(msg.data)
