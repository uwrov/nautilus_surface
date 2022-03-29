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
        self.current_state = None
        self.launch_continuous_publisher()

    def update_state(self, state):
        if (self.current_state is None or state != self.current_state):
            if (state["ang_x"] != 0 or state["ang_y"] != 0 or state["ang_z"] != 0):
                state["lin_x"] = 0
                state["lin_y"] = 0
                state["lin_z"] = 0

            self.msg.force.x = state["lin_x"]
            self.msg.force.y = state["lin_y"]
            self.msg.force.z = state["lin_z"]
            self.msg.torque.x = state["ang_x"]
            self.msg.torque.y = state["ang_y"]
            self.msg.torque.z = state["ang_z"]

            self.current_state = state

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
