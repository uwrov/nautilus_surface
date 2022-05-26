#!/usr/bin/env python3

from locks.lock_service_client import LockClient
import time
import rospy

if __name__ == '__main__':
    rospy.init_node('test_lock_client')
    rospy.on_shutdown(lambda: rospy.loginfo("shutting down test client"))

    # Test case 1 : manage lock using the with statement
    with LockClient('test_client') as lc:
        time.sleep(0.25)
        rospy.loginfo("lock acquired automatically")
    rospy.loginfo("lock released automatically")


    # Test case 2 : manage lock manually
    lc = LockClient('test_client')
    lc.lock()
    rospy.loginfo("lock acquired manually")
    lc.unlock()
    rospy.loginfo("lock released manually")
