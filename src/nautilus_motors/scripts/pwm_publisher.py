#!/usr/bin/env python3
import rospy
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Wrench
from PWMCalculator.pwm_calculator import PWMCalculator

"""
ROS motor code driver
"""

listen_topic = '/nautilus/motors/commands'
publish_topic = '/nautilus/motors/pwm'

dims = [MultiArrayDimension('data', 6, 16)]
layout = MultiArrayLayout(dim=dims, data_offset=0)

# Calculate pwm to be applied onto each motor and publish to motors
def drive(wrench_msg, args):
    pwm_output = args[0](wrench_msg)
    msg = Int16MultiArray(layout=layout, data=pwm_output)
    args[1](msg)

# launch publisher and subscriber
def main():
    print('starting publisher on', publish_topic)
    pub = rospy.Publisher(publish_topic, Int16MultiArray, queue_size=1)

    print('starting listener on', listen_topic)
    controller = PWMCalculator()
    rospy.init_node('motors')
    rospy.Subscriber(listen_topic, Wrench, drive,
                     (controller.convert_vector_to_pwms, pub.publish))

    rospy.on_shutdown(shutdown_fn)
    rospy.spin()

def shutdown_fn():
    print('shutting down')

if __name__ == '__main__':
    main()
