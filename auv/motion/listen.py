import rospy
from mavros_msgs.msg import OverrideRCIn

def override_callback(data):
    # This function will be called whenever new data is received on the /mavros/rc/override topic
    rospy.loginfo(rospy.get_caller_id() + " Override RC In: %s", data.channels)

def override_listener():
    rospy.init_node('override_listener', anonymous=True)
    rospy.Subscriber("/mavros/rc/override", OverrideRCIn, override_callback)
    rospy.loginfo("Override listener node initialized.")
    rospy.spin()  # Keeps the node running until it is shutdown

if __name__ == '__main__':
    try:
        override_listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down override listener node.")
