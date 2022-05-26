#!/usr/bin/env python3

from threading import Lock
from robot_module.srv import acquire_lock, release_lock
from std_msgs.msg import String, Bool
import rospy

ui_enablement_topic = '/ui/enablement'

class LockServer:
    def __init__(self):
        self.lock_holder = None
        self.ui_publisher = None
        self.mutex = Lock


    def handle_acquire_request(self, req):
        self.mutex.acquire()
        req_name = req.requester_name

        # If no autonomous script is holding on, preempt our UI
        if (self.lock_holder is None):
            self.lock_holder = req_name
            self.ui_publisher.publish(False)


    def handle_release_request(self, req):
        self.mutex.acquire()
        req_name = req.requester_name

        #  If the client holding the lock is releasing, give control back to UI
        if (self.lock_holder is not None and req_name == self.lock_holder):
            self.lock_holder = None
            self.ui_publisher.publish(True)


    def init_server(self):
        rospy.init_node('lock_server')
        rospy.on_shutdown(lambda : rospy.loginfo("Shutting down Lock Server"))

        rospy.loginfo("Starting Lock Server")
        self.ui_publisher = rospy.Publisher(ui_enablement_topic, Bool, queue_size=1)

        acquire_service = rospy.Service('acquire_service', acquire_lock, self.handle_acquire_request, self)
        release_service = rospy.Service('release_service', release_lock, self.handle_release_request, self)

        rospy.spin()