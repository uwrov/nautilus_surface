#!/usr/bin/env python3
import rospy
import os
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

def main():
  DIM=(640, 480)
  K=np.array([[232.59834329560857, 0.0, 323.69056666007043], [0.0, 224.2212045670311, 249.45648108995732], [0.0, 0.0, 1.0]])
  D=np.array([[-0.030436347449448193], [0.009118483708036189], [-0.03442166148483811], [0.014312258830753885]])

  ips_to_topics = {'http://192.168.0.99:8081' : '/rov_camera/front',
                   'http://192.168.0.99:8082' : '/rov_camera/down',
                   #'http://192.168.0.99:8083' : '/rov_camera/left',
                   #'http://192.168.0.99:8084' : '/rov_camera/right'
                   }

  rospy.init_node('motion')
  pairs = {}
  rospy.on_shutdown(lambda: shutdown_fn([pairs]))
  rate = rospy.Rate(60) # framerate - consider lowering

  for ip in ips_to_topics:
    stream = cv2.VideoCapture(ip)
    publisher = rospy.Publisher(ips_to_topics[ip], CompressedImage, queue_size=1)
    if (ips_to_topics[ip] == '/rov_camera/front'):
      pairs[stream] = (publisher, False)
    else:
      pairs[stream] = (publisher, False)

  msg = CompressedImage()

  while not rospy.is_shutdown():
    for stream in pairs:
      try:
          ret, frame = stream.read()
          if pairs[stream][1]:
            (h, w) = frame.shape[:2]
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
            undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            msg.data = undistorted_img
          else:
            msg.data = frame

          if ret:
            pairs[stream][0].publish(msg)
      except Exception as e:
          print(e)
          pass
    rate.sleep()

def shutdown_fn(pairs_ptr):
  pairs = pairs_ptr[0]
  for stream in pairs:
    stream.release()
  print('shutting down')

if __name__ == '__main__':
    main()
