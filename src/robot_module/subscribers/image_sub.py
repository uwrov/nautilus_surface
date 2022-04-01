import rospy
from .subscriber import ServerSub

class ImageSub(ServerSub):
    """
    Subscriber which receives image data from a ROS camera.
    """

    def __init__(self, topic):
        super().__init__(topic, CompressedImage)
        self.image_cache = [] # TODO: Cache last 10 frames

    def callback(self, msg):
        pass
        # TODO: Push msg.data onto cache (only 10 frames in cache)

    # TODO: getCurrentFrame -> returns last cached frame
    # TODO: getAllFrames -> Returns cache