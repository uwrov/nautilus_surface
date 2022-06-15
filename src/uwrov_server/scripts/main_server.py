#!/usr/bin/env python3
import signal
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit

from robot_module import RobotModule

HOST_IP = "0.0.0.0"
HOST_PORT = 4040


app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*", async_mode='gevent')

robot = None
'''
# Maps socketio image id to ROS topic name
image_handles = ['camera_stream', 'img_sub']

# aux storage to make sure subscriber objects aren't garbage collected
subscribers = {
    'camera_h': SubInfo('/nautilus/cameras/stream', 'Image Display', 'camera_stream', None),
    'img_h': SubInfo('/image/distribute', 'Image Display', 'img_sub', None),
}

# Map of handles to rospy pub objects
publishers = {
    'move_h': PubInfo('/nautilus/motors/vector', None),
}
'''

@sio.on("Get IDs")
def send_image_id():
    #sio.emit("IDs", {'ids': image_handles}, broadcast=True)
    pass


@sio.on("Send Movement")
def send_move_state(data):
    robot.request_priority()
    robot.set_vel(data["linear"], data["angular"])
    #publishers['move_h'].pub.update_state(data)


@sio.on("Set Manipulator Angle")
def send_move_state(data):
    robot.set_mani_angle(data)


@sio.on("Set Manipulator Velocity")
def send_move_state(data):
    robot.set_mani_vel(data)


@sio.on("Stop Manipulator")
def send_move_state():
    robot.stop_mani()


@sio.on("Arm Motors")
def arm_motors(data):
    print("arming motors")
    robot.arm_motors()


def shutdown_server(signum, frame):
    sio.stop()
    exit(signal.SIGTERM)


if __name__ == '__main__':
    """ Sets up rospy and starts servers """
    robot = RobotModule("surface")
    robot.setup("movement")
    robot.setup("manipulator")

    # Define a way to exit gracefully
    signal.signal(signal.SIGINT, shutdown_server)

    # Start sio server
    sio.run(app, host=HOST_IP, port=HOST_PORT)
