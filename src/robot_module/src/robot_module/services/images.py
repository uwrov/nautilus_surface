# Image Module that contains all the logic for robot Image requests

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from .service import Service


image_topics = [
    '/rov_camera/front',
    '/rov_camera/down',
    '/rov_camera/left',
    '/rov_camera/right'
    ]


class RobotImages(Service):
    """
    Functionalities related to the motors on the robot.
    """
    def __init__(self, node_name):
        super().__init__(node_name)

        self.images = dict.fromkeys(image_topics, None)
        self.callbacks = []
        self.subs = {}

        for t in image_topics:
            self.subs[t] = rospy.Subscriber(t, CompressedImage,
                                            self._handle_callbacks,
                                            (t),
                                            queue_size=1)

    def setup(self):
        pass

    def _handle_callbacks(self, image, topic):
        image = image.data				# A bad thing but quick fix
        self._update_image(image, topic)
        for func in self.callbacks:
            try:
                func(image, topic)
            except Exception as e:
                print(e)

    def _update_image(self, image, topic):
        self.images[topic] = image


    def get_image(self, topic):
        try:
            return self.images[topic]
        except Exception as e:
            print(e)
            return None


    def get_image_topics(self):
        return image_topics
