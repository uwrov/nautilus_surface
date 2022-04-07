# Movement Module that contains all the logic for robot Movement

#!/usr/bin/env python3


import rospy
import threading
from .service import Service

from geometry_msgs.msg import Wrench
from std_msgs.msg import String


topic = "/nautilus/motors/commands"
lock_topic = "/nautilus/motors/lock"


class Movement(Service):
    """
    Publisher for movement commands, dictated by current state of the xbox controller
    """
    def __init__(self, node_name):
        super().__init__(node_name)
        self.msg = Wrench()

        self.lock_pub = rospy.Publisher(lock_topic, String, queue_size=1)
        self.lock_sub = rospy.Subscriber(lock_topic, String,
                                            self.__update_priority,
                                            queue_size=1)
        self.priority_node = node_name

        self.move_pub = rospy.Publisher(topic, Wrench, queue_size=10)


    def setup(self):
        self.launch_continuous_publisher()


    # Movement Priority Determinants
    def request_priority(self):
        self.lock_pub.publish(self.node_name)


    def has_priority(self):
        return True #self.priority_node == self.node_name


    def __update_priority(self, prio_node):
        self.priority_node = prio_node.data


    # Movement Logic
    def set_velocity(self, linear, angular=[0, 0, 0]):
        if self.has_priority():
            self.msg.force.x = linear[0]
            self.msg.force.y = linear[1]
            self.msg.force.z = linear[2]
            self.msg.torque.x = angular[0]
            self.msg.torque.y = angular[1]
            self.msg.torque.z = angular[2]
            print("moving")
            print(self.msg)


    def publish(self):
        self.move_pub.publish(self.msg)
        print(self.msg)


    def publish_continuous(self, rate: int):
        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            self.publish()
            r.sleep()


    def launch_continuous_publisher(self):
        self.publisher_thread = threading.Thread(
            target=self.publish_continuous, args=(20,), daemon=True)
        self.publisher_thread.start()
