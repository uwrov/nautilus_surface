#!/usr/bin/env python3

from locks.lock_service_client import LockClient
import rospy

if __name__ == '__main__':
    rospy.init_node('test_lock_client')
    with LockClient('test_client') as lc:
        rospy.loginfo("lock acquired")
    rospy.loginfo("lock released")

    lc = LockClient('test_client')
    lc.lock()
    lc.unlock()