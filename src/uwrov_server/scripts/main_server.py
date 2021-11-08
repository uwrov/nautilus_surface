#!/usr/bin/env python3
import rospy
import threading
import signal
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit

from publishers.channel_pub import ChannelPub
from publishers.move_pub import MovePub
from publishers.user_webcam_pub import UserWebcamPub

from subscribers.image_sub import ImageSub

HOST_IP = "0.0.0.0"
HOST_PORT = 4040


class SubInfo:
    def __init__(self, ros_topic, sio_route, sio_id, sub):
        self.ros_topic = ros_topic
        self.sio_route = sio_route
        self.sio_id = sio_id
        self.sub = sub


class PubInfo:
    def __init__(self, ros_topic, pub):
        self.ros_topic = ros_topic
        self.pub = pub


app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*", async_mode='gevent')


# Maps socketio image id to ROS topic name
image_handles = ['camera_stream', 'img_sub']

# aux storage to make sure subscriber objects aren't garbage collected
subscribers = {
    'camera_h': SubInfo('/nautilus/cameras/stream', 'Image Display', 'camera_stream', None),
    'img_h': SubInfo('/image/distribute', 'Image Display', 'img_sub', None)
}

# Map of handles to rospy pub objects
publishers = {
    'move_h': PubInfo('/nautilus/motors/commands', None),
    'channel_h': PubInfo('/nautilus/cameras/switch', None),
    'user_webcam_h': PubInfo('/nautilus/cameras/user_webcam', None)
}


@sio.on("Get IDs")
def send_image_id():
    sio.emit("IDs", {'ids': image_handles}, broadcast=True)


@sio.on("Set Camera")
def set_image_camera(cam_num):
    publishers['channel_h'].pub.publish(cam_num)


@sio.on("Send State")
def send_move_state(data):
    publishers['move_h'].pub.update_state(data)

@sio.on("Send User Webcam Frame")
def send_user_webcam(blob):
    publishers['user_webcam_h'].pub.update_video_frame(blob)


def shutdown_server(signum, frame):
    rospy.loginfo("Shutting down main server")
    sio.stop()
    exit(signal.SIGTERM)


if __name__ == '__main__':
    """ Sets up rospy and starts servers """
    rospy.loginfo("main server is running")

    rospy.init_node('surface', log_level=rospy.DEBUG)

    # Register our subscribers and publishers
    for handle in ['camera_h', 'img_h']:
        subinfo = subscribers[handle]
        subinfo.sub = ImageSub(
            subinfo.ros_topic, subinfo.sio_route, subinfo.sio_id, sio)

    publishers['channel_h'].pub = ChannelPub(publishers['channel_h'].ros_topic)
    publishers['move_h'].pub = MovePub(publishers['move_h'].ros_topic)
    publishers['user_webcam_h'].pub = UserWebcamPub(publishers['user_webcam_h'].ros_topic)

    # Define a way to exit gracefully
    signal.signal(signal.SIGINT, shutdown_server)

    # Start the ROS services and sio server
    threading.Thread(target=rospy.spin, daemon=True).start()
    sio.run(app, host=HOST_IP, port=HOST_PORT)
