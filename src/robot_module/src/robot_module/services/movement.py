# Movement Module that contains all the logic for robot Movement

#!/usr/bin/env python3


import rospy
import threading
from .service import Service

from geometry_msgs.msg import Wrench
from std_msgs.msg import String


topic = "/nautilus/motors/commands"
lock_topic = "/nautilus/motors/lock"
arm_topic = "/nautilus/motors/arm"


class Movement(Service):
    """
    Functionalities related to the motors on the robot.
    """
    def __init__(self, node_name):
        super().__init__(node_name)
        self.msg = Wrench()
        
        self.arm_pub = rospy.Publisher(arm_topic, String, queue_size=1)

        self.lock_pub = rospy.Publisher(lock_topic, String, queue_size=1)
        self.lock_sub = rospy.Subscriber(lock_topic, String,
                                            self.__update_priority,
                                            queue_size=1)
        self.priority_node = node_name

        self.move_pub = rospy.Publisher(topic, Wrench, queue_size=10)


    def setup(self):
        pass


    # Movement Priority Determinants
    def request_priority(self):
        self.lock_pub.publish(self.node_name)


    def has_priority(self):
        return True #self.priority_node == self.node_name


    def __update_priority(self, prio_node):
        self.priority_node = prio_node.data


    # Publishing movement commands to ROS
    def set_velocity(self, linear, angular=[0, 0, 0]):
        if self.has_priority():
            self.msg.force.x = linear[0]
            self.msg.force.y = linear[1]
            self.msg.force.z = linear[2]
            self.msg.torque.x = angular[0]
            self.msg.torque.y = angular[1]
            self.msg.torque.z = angular[2]
            self.publish()


    def arm(self):
        self.arm_pub.publish("arm")


    def publish(self):
        self.move_pub.publish(self.msg)
