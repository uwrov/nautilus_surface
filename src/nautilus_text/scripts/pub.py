import rospy
from std_msgs.msg import String

def main():
  pub = rospy.Publisher('/nautilus/text', String, queue_size=10)
  rospy.init_node('text_publisher', log_level=rospy.DEBUG)
  while not rospy.is_shutdown():
    msg = input("Enter message: ")
    pub.publish(msg)

if __name__ == '__main__':
  main()