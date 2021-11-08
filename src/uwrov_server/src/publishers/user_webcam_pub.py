#!/usr/bin/env python3
import rospy
import threading
from .publisher import ServerPub
from sensor_msgs.msg import CompressedImage


class UserWebcamPub(ServerPub):
    """
    Publisher for user webcam frames.
    """

    def __init__(self, topic):
        super().__init__(topic, CompressedImage, queue_size=1)
        self.msg = CompressedImage()
        self.publisher_thread = None

        self.launch_continuous_publisher()

    def update_video_frame(self, blob):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.format = "jpeg"
        self.msg.data = blob

    def publish(self, **kwargs):
        self.publisher.publish(self.msg)

    def publish_continuously(self, state):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def launch_continuous_publisher(self):
        self.publisher_thread = threading.Thread(target=self.publish_continuously, args=(self,), daemon=True)
        self.publisher_thread.start()
