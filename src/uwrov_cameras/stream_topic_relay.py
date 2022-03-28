#!/usr/bin/env python3
import picamera
import rospy
import os
from sensor_msgs.msg import CompressedImage
import cv2

def main():
  ips_to_topics = {'192.168.0.99:8081' : '/nautilus/motion1',
                   '192.168.0.99:8082' : '/nautilus/motion2'}

  rospy.init_node('motion')
  pairs = {}
  rospy.on_shutdown(shutdown_fn, [pairs])
  rate = rospy.Rate(60) # framerate - consider lowering
  
  for ip in ips_to_topics:
    stream = cv2.VideoCapture(ip)
    publisher = rospy.Publisher(ips_to_topics[ip], CompressedImage, queue_size=1)
    pairs[stream] = publisher

  msg = CompressedImage()

  while not rospy.is_shutdown():
    for stream in pairs:
      ret, frame = stream.read()
      msg.data = frame
      if ret:
        pairs[stream].publish(msg)
    rate.sleep()

def shutdown_fn(pairs_ptr):
  pairs = pairs_ptr[0]
  for stream in pairs:
    stream.release()
  print('shutting down')

if __name__ == '__main__':
    main()