#!/usr/bin/env python3

from threading import Lock
from robot_module.srv import acquire_lock, release_lock
from std_msgs.msg import String, Bool
import rospy

# Consider tossing this into a command line parameter for on-the-fly retargeting
ui_enablement_topic = '/ui/enablement'

class LockServer:
    def __init__(self):
        self.lock_holder = None
        self.ui_publisher = None
        self.mutex = Lock()


    def handle_acquire_request(self, req):
        with self.mutex:
            req_name = req.requester

            # If no autonomous script is holding on, preempt our UI
            if self.lock_holder is None:
                self.lock_holder = req_name
                self.ui_publisher.publish(False)
                return True

            # Be forgiving and allow a single caller to acquire a lock more than once
            return self.lock_holder == req_name


    def handle_release_request(self, req):
        with self.mutex:
            req_name = req.requester

            #  If the client holding the lock is releasing, give control back to UI
            if self.lock_holder is not None and req_name == self.lock_holder:
                self.lock_holder = None
                self.ui_publisher.publish(True)
                return True

            return False


    def init_server(self):
        rospy.init_node('lock_server')
        rospy.on_shutdown(lambda : rospy.loginfo("Shutting down Lock Server"))

        rospy.loginfo("Starting Lock Server")
        self.ui_publisher = rospy.Publisher(ui_enablement_topic, Bool, queue_size=1)

        # services can't take in extra parameters like a subscriber can,
        # so we have to use this funny workaround
        # https://answers.ros.org/question/247540/pass-parameters-to-a-service-handler-in-rospy/
        acquire_service = rospy.Service('acquire_service', acquire_lock, lambda msg: self.handle_acquire_request(msg))
        release_service = rospy.Service('release_service', release_lock, lambda msg: self.handle_release_request(msg))

        rospy.spin()
