#!/usr/bin/env python3
import rospy
import threading
from .publisher import ServerPub
from geometry_msgs.msg import Wrench


class MovePub(ServerPub):
    """
    Publisher for movement commands, dictated by current state of the xbox controller
    """

    def __init__(self, topic):
        super().__init__(topic, Wrench, queue_size=10)
        self.msg = Wrench()
        self.launch_continuous_publisher()

    def set_velocity(self, linear, angular=[0, 0, 0]):
        self.msg.force.x = linear[0]
        self.msg.force.y = linear[1]
        self.msg.force.z = linear[2]
        self.msg.torque.x = angular[0]
        self.msg.torque.y = angular[1]
        self.msg.torque.z = angular[2]

    def publish(self):
        self.publisher.publish(self.msg)

    def publish_continuous(self, rate: int):
        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            self.publish()
            r.sleep()

    def launch_continuous_publisher(self):
        self.publisher_thread = threading.Thread(
            target=self.publish_continuous, args=(20,), daemon=True)
        self.publisher_thread.start()
