#!/usr/bin/env python3
import rospy
import threading
import signal
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit

from publishers.channel_pub import ChannelPub
from publishers.move_pub import MovePub

from subscribers.image_sub import ImageSub

HOST_IP = "0.0.0.0"
HOST_PORT = 4040

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*", async_mode='gevent')

channel_publisher = None

# Maps socketio image id to ROS topic name
image_handles = {
    "camera_stream": "/nautilus/cameras/stream",
    "img_sub": "/image/distribute"
}

# aux storage to make sure subscriber objects aren't garbage collected
subscribers = []

# Map of handles to rospy pub objects
publishers = {
    'move_h': None,
    'channel_h': None
}


@sio.on("Get IDs")
def send_image_id():
    sio.emit("IDs", {'ids': list(image_handles.keys())}, broadcast=True)


@sio.on("Set Camera")
def set_image_camera(cam_num):
    publishers['channel_h'].publish(cam_num)


@sio.on("Send State")
def send_move_state(data):
    publishers['move_h'].update_state(data)


def shutdown_server(signum, frame):
    rospy.loginfo("Shutting down main server")
    sio.stop()
    exit(signal.SIGTERM)


if __name__ == '__main__':
    """ Sets up rospy and starts servers """
    rospy.loginfo("main server is running")

    rospy.init_node('surface', log_level=rospy.DEBUG)

    # Register our subscribers and publishers
    for sio_id, ros_topic in image_handles.items():
        subscribers.append(ImageSub(ros_topic, sio_id))

    publishers['channel_h'] = ChannelPub('/nautilus/cameras/switch')
    publishers['move_h'] = MovePub('/nautilus/motors/commands')

    # Define a way to exit gracefully
    signal.signal(signal.SIGINT, shutdown_server)

    # Start the ROS services and sio server
    threading.Thread(target=rospy.spin, daemon=True).start()
    sio.run(app, host=HOST_IP, port=HOST_PORT)
