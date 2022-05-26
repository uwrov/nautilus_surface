#!/usr/bin/env python3

from robot_module.srv import acquire_lock, release_lock
from std_msgs.msg import String
import rospy
import time

class LockClient:
    def __init__(self, name):
        self.name = name


    def __enter__(self):
        acquire_attempts = 0
        lock_acquired = self.lock()

        # use exponential backoff to try again without flooding network
        while not lock_acquired:
            rospy.loginfo("Couldn't acquire the lock, trying again")
            time.sleep(1 * (2**acquire_attempts))
            acquire_attempts += 1

            lock_acquired = self.lock()


    def __exit__(self, exc_type, exc_value, traceback):
        self.unlock()


    def lock(self) -> bool:
        rospy.wait_for_service('acquire_service')
        try:
            acquire_lock_fn = rospy.ServiceProxy('acquire_service', acquire_lock)
            lock_server_response = acquire_lock_fn(self.name)
            return lock_server_response.result
        except rospy.ServiceException as e:
            rospy.logerr("couldn't access acquire_service service")
            return False


    def unlock(self):
        rospy.wait_for_service('release_service')
        try:
            release_lock_fn = rospy.ServiceProxy('release_service', release_lock)
            release_lock_fn(self.name)
        except rospy.ServiceException as e:
            rospy.logerr("couldn't access release_service service... something's broken, destroy everything and try again")
