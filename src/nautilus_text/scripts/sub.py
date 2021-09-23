import rospy
from std_msgs.msg import String

def callback(msg):
  rospy.loginfo(msg.data)

def main():
  rospy.init_node('text_server', log_level=rospy.DEBUG)
  rospy.Subscriber('/nautilus/text', String, callback)
  rospy.spin()

if __name__ == '__main__':
  main()