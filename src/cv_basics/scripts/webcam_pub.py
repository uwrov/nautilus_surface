import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def publish_message():
    # Create a publisher node of type Image
    pub = rospy.Publisher('video_frames', Image, queue_size=10)

    # Initialize the node in rospy
    rospy.init_node('webcam_pub', anonymous=True)

    # Set capture rate to 10hz
    rate = rospy.Rate(10)

    # Create a VideoCapture object, using default device
    cap = cv2.VideoCapture(0)

    # Create a CvBridge object to convert ROS image to OpenCV image
    br = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if ret:
            # Log successful capture
            rospy.loginfo("Publishing video frame")

            # Publish image
            pub.publish(br.cv2_to_imgmsg(frame))

        rate.sleep()

if __name__ == "__main__":
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass