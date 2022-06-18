# Manipulator Module that contains all the logic for robot Movement

#!/usr/bin/env python3


import rospy
import time
import threading
from .service import Service

from std_msgs.msg import Float32


topic = "/nautilus/manipulator/pwm"

range = (0, 100)


class Manipulator(Service):
    """
    Functionalities related to the motors on the robot.
    """
    def __init__(self, node_name):
        super().__init__(node_name)
        self.msg = Float32()

        self.mani_pub = rospy.Publisher(topic, Float32, queue_size=10)

        self.thread = None
        self.running = False
        self.velocity = 0


    def setup(self):
        pass


    # set the amount of open for the manipulator by percent
    def set_angle(self, percent):
        self.msg.data = percent
        self.publish()

    def set_velocity(self, vel):
        if vel != self.msg.data: self.stop_moving()
        self.running = True
        self.thread = threading.Thread(target=self.run, args=(lambda : self.running, ))
        self.thread.start()

    def stop_moving(self):
        print("trying to stop")
        if self.thread is not None:
            self.running = False
            self.thread.join()
            self.thread = None
            print("stop")

    def run(self, is_running):
        while is_running:
            self.msg.data += self.velocity
            self.publish()
            print("updating")
            time.sleep(0.05)


    def _throttle(self):
        if self.msg.data < range[0]: self.msg.data = range[0]
        if self.msg.data > range[1]: self.msg.data = range[1]


    def publish(self):
        self._throttle()
        print("publishing")
        self.mani_pub.publish(self.msg)
