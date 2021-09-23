import rospy
from std_msgs.msg import String

def main():
  rospy.init_node('text_publisher_constant', log_level=rospy.DEBUG)
  pub = rospy.Publisher('/nautilus/text', String, queue_size=10)
  rate = rospy.Rate(60)
  while not rospy.is_shutdown():
    msg = str(rospy.get_time())
    pub.publish(msg)
    rate.sleep()

if __name__ == '__main__':
  main()