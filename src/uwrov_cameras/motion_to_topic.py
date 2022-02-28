#!/usr/bin/env python3
import picamera
import rospy
import os
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

def main():
  src = 'ip'
  stream = cv2.VideoCapture(src)
  rospy.init_node('motion')
  rospy.on_shutdown(shutdown_fn)

  publisher = rospy.Publisher('/nautilus/motion', Image, queue_size=1)

  rate = rospy.Rate(100)
  br = CvBridge()

  while not rospy.is_shutdown():
        ret, frame = stream.read()
        if ret:
          publisher.publish(br.cv2_to_imgmsg(frame))
        rate.sleep()

def shutdown_fn():
  stream.release()
  print('shutting down')

if __name__ == '__main__':
    main()