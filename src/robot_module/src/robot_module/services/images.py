# Image Module that contains all the logic for robot Image requests

#!/usr/bin/env python3

import rospy
from .service import Service




class RobotImages(Service):
    """
    Functionalities related to the motors on the robot.
    """
    def __init__(self, node_name):
        super().__init__(node_name)


    def setup(self):
        pass
