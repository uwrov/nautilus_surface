import rospy
from .subscriber import ServerSub
from flask_socketio import SocketIO
from sensor_msgs.msg import CompressedImage


class ImageSub(ServerSub):
    """
    Subscriber which receives image data from a ROS camera.
    """

    def __init__(self, topic, sio_route, sio_id, sio):
        super().__init__(topic, CompressedImage)
        self.sio_route = sio_route
        self.sio_id = sio_id
        self.sio = sio

    def callback(self, msg):
        rospy.loginfo("emitting data from ImageSub")
        packet = {
            'image': msg.data,
            'id': self.sio_id
        }
        self.sio.emit(self.sio_route, packet, broadcast=True)
