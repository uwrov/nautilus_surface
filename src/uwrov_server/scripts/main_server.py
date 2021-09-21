#!/usr/bin/env python3
import rospy
import threading
import signal
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int16, Empty
from geometry_msgs.msg import Wrench
from subscribers import image_sub
from publishers import move_pub
from subscribers.text_sub import TextServer

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
    image_sub.send_ids(sio)

@sio.on("Set Camera")
def set_image_camera(data):
    image_sub.set_camera(data, channel_publisher)

@sio.on("Send State")
def send_move_state(data):
    move_pub.update_state(data, sio)

@sio.on("Activate Script")
def publish_empty_signal():
    empty_publisher.publish(msg)

def shutdown_server(signum, frame):
    rospy.loginfo("Shutting down main server")
    sio.stop()
    exit(signal.SIGTERM)

if __name__ == '__main__':
    """ Sets up rospy and starts servers """
    rospy.loginfo("main server is running")

    rospy.init_node('surface', log_level=rospy.DEBUG)
    image_subscriber = rospy.Subscriber(image_sub.topics['img_sub'], CompressedImage, image_sub.send_image, ('img_sub', sio))
    camera_subscriber = rospy.Subscriber(image_sub.topics['camera_stream'], CompressedImage, image_sub.send_image, ('camera_stream', sio))

    velocity_publisher = rospy.Publisher('/nautilus/motors/commands', Wrench, queue_size=10)
    channel_publisher = rospy.Publisher('/nautilus/cameras/switch', Int16, queue_size=1)
    threading.Thread(target=move_pub.publish, args=(velocity_publisher,), daemon=True).start()

    text = TextServer("/nautilus/text")
    threading.Thread(target=rospy.spin, daemon=True).start()
    signal.signal(signal.SIGINT, shutdown_server)

    sio.run(app, host=HOST_IP, port=HOST_PORT)
