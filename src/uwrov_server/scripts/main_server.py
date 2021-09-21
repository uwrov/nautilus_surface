#!/usr/bin/env python3
import rospy
import threading
import signal
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int16, Empty
from geometry_msgs.msg import Wrench
from servers import image_server
from servers import move_server
from servers import scripts_server

# from engineio.payload import Payload
# Payload.max_decode_packets = 50


# HOST_IP = "localhost"
HOST_IP = "0.0.0.0"
HOST_PORT = 4040

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*", async_mode='gevent')

channel_publisher = None
scripts_manager = None
empty_publisher =None
msg = Empty()

@sio.on("Get IDs")
def send_image_id():
    image_server.send_ids(sio)

@sio.on("Set Camera")
def set_image_camera(data):
    image_server.set_camera(data, channel_publisher)

@sio.on("Send State")
def send_move_state(data):
    move_server.update_state(data, sio)

@sio.on('Get Scripts')
def send_scripts_list():
    scripts_server.send_scipts()

@sio.on("Send Commands")
def send_script_command(data):
    scripts_server.json_request(data)

@sio.on("Error Message")
def send_error_message(data):
    scripts_server.process_error_msg(data)

@sio.on("Activate Script")
def publish_empty_signal():
    empty_publisher.publish(msg)

def shutdown_server(signum, frame):
    rospy.loginfo("Shutting down main server")
    sio.stop()
    exit(signal.SIGTERM)

if __name__ == '__main__':
    """ Sets up rospy and starts servers """
    rospy.logdebug("main server is running")

    rospy.init_node('surface', log_level=rospy.DEBUG)
    image_subscriber = rospy.Subscriber(image_server.topics['img_sub'], CompressedImage, image_server.send_image, ('img_sub', sio))
    camera_subscriber = rospy.Subscriber(image_server.topics['camera_stream'], CompressedImage, image_server.send_image, ('camera_stream', sio))

    old_subscriber_1 = rospy.Subscriber(image_server.topics['old cam 1'], CompressedImage, image_server.send_image, ('old cam 1', sio))
    old_subscriber_2 = rospy.Subscriber(image_server.topics['old cam 2'], CompressedImage, image_server.send_image, ('old cam 2', sio))

    velocity_publisher = rospy.Publisher('/nautilus/motors/commands', Wrench, queue_size=10)
    channel_publisher = rospy.Publisher('/nautilus/cameras/switch', Int16, queue_size=1)
    threading.Thread(target=move_server.publish, args=(velocity_publisher,), daemon=True).start()

    scripts_manager = scripts_server.ScriptManager(sio)

    empty_publisher = rospy.Publisher('/nautilus/controls/signal', Empty, queue_size=1)

    signal.signal(signal.SIGINT, shutdown_server)
    # imgsub.register(image_server.topics['img_sub'], CompressedImage, sio)
    sio.run(app, host=HOST_IP, port=HOST_PORT)
