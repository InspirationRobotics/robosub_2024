import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2


bridge = CvBridge()

rospy.init_node("MotionTest", anonymous=True)
def callbacks(msg):
    # check if a msg is convertable?
    try:
        # Convert ROS Image to OpenCV image (BGR8 encoding)
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"cv_bridge error: {e}")
        return

sub = rospy.Subscriber('/auv/camera/videoOAKdRawForward', Image, callbacks)

rospy.spin()




