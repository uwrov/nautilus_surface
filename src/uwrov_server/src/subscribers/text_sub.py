import rospy
from std_msgs.msg import String
from flask_socketio import SocketIO

from .subscriber import SeverSub


class TextSub(SeverSub):
    """
    Gets text from a topic.
    """

    def __init__(self, topic, sio_route, sio_id):
        super().__init__(topic, String)
        self.sio_route = sio_route
        self.sio_id = sio_id
        self.sio = SocketIO()

    def callback(self, msg):
        packet = {
            'text': msg.data,
            'id': self.id
        }
        self.sio.emit(self.sio, packet, broadcast=True)
