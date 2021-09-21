from .subscriber import ServerSub
from flask_socketio import SocketIO, emit
from sensor_msgs.msg import CompressedImage


class ImageSub(ServerSub):
    """
    Subscriber which receives image data from a ROS camera.
    """

    def __init__(self, topic, sio_route):
        super().__init__(topic, CompressedImage)

        self.sio_route = sio_route
        self.sio = SocketIO()

    def callback(self, msg):
        packet = {
            'image': msg.data,
            'id': self.id
        }
        self.sio.emit(self.sio_route, packet, broadcast=True)
