from .subscriber import ServerSub
from flask_socketio import SocketIO, emit
from sensor_msgs.msg import CompressedImage


class ImageSub(ServerSub):
    """
    Subscriber which receives image data from a ROS camera.
    """

    def __init__(self, topic, id):
        super().__init__(topic, CompressedImage)

        self.id = id
        self.sio = SocketIO()

    def callback(self, msg):
        packet = {
            'image': msg.data,
            'id': self.id
        }
        self.sio.emit("Image Display", packet, broadcast=True)
