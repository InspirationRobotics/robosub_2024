
import rospy
import time
from auv.utils.img_bridge import imgmsg_to_cv2, cv2_to_imgmsg
from sensor_msgs.msg import Image
import cv2

rospy.init_node("cv_debugger", anonymous=True)

def callbacks(msg):
    try:
        cv_image = imgmsg_to_cv2(msg)
        rospy.loginfo('success')
    except Exception as e:
        rospy.logerr(e)
        return
    cv2.waitKey(1)

sub = rospy.Subscriber('/auv/camera/videoOAKdRawForward', Image, callbacks)
rospy.spin()
cv2.destroyAllWindows()
