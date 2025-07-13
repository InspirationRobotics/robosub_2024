#!/usr/bin/env python2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

bridge = CvBridge()
rospy.init_node("cv_debugger", anonymous=True)

def callbacks(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"cv_bridge error: {e}")
        return
    cv2.imshow("OAK-D forward", cv_image)
    cv2.waitKey(1)

sub = rospy.Subscriber('/auv/camera/videoOAKdRawForward', Image, callbacks)
rospy.spin()
cv2.destroyAllWindows()
