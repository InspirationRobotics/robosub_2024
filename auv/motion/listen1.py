import rospy
from std_msgs.msg import String

def callback(data):
    print(f"Received: {data.data}")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/mavros/rc/override", String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
