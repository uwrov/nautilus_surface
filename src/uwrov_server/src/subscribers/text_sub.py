import rospy
from std_msgs.msg import String

from .subscriber import SubServer

class TextServer(SubServer):
  def __init__(self, topic):
    super(TextServer, self).__init__(topic, String)

  def callback(self, msg):
    rospy.loginfo(msg.data)
