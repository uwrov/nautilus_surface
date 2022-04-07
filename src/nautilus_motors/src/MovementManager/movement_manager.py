import rospy
from geometry_msgs.msg import Wrench
from std_msgs.msg import String

listen_topic = "/nautilus/motors/commands"
publish_topic = "/nautilus/motors/actions"

lock_topic = "nautilus/motors/lock_owner"

priority_node = ""

class MovementManager():
    def __init__(self):
        rospy.init_node("Movement Manager", log_level=rospy.DEBUG)
        
        # Command subscriber
        rospy.Subscriber(listen_topic, Wrench, self.callback, queue_size=10)
        
        # ROS Publishers
        self.motor_pub = rospy.Publisher(publish_topic, Wrench, queue_size=1)
        self.lock_pub = rospy.Publisher(lock_topic, String, queue_size=1)

    
    def callback(self, msg):
        # See if source == priority_node
        pass

    def manage_lock(self):
        pass