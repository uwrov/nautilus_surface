import rospy
from std_msgs.msg import String

def main():
  rospy.init_node('text_publisher', log_level=rospy.DEBUG)
  pub = rospy.Publisher('/nautilus/text', String, queue_size=10)
  while not rospy.is_shutdown():
    msg = input("Enter message: ")
    pub.publish(msg)

if __name__ == '__main__':
  main()