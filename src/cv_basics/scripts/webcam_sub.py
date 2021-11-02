import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def callback(data):
    # Convert ROS image to OpenCV image
    br = CvBridge()

    rospy.loginfo("Receiving video frame")

    current_frame = br.imgmsg_to_cv2(data)

    cv2.imshow("Camera", current_frame)

    cv2.waitKey(1)

def receive_message():
    rospy.init_node("video_sub", anonymous=True)

    rospy.Subscriber("video_frames", Image, callback)

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    receive_message()