from robot_module import RobotModule
import rospy
import numpy as np
import cv2

from scipy.ndimage import filters

from sensor_msgs.msg import CompressedImage

image_topic = "/???"

robot = RobotModule("auto_drive")


def make_decisions(image):
    np_arr = np.fromstring(image.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

    
    pass



def main():
  image_sub = rospy.Subscriber(image_topic,
                                CompressedImage,
                                make_decisions,
                                queue_size=1)


if __name__ == '__main__':
    main()
