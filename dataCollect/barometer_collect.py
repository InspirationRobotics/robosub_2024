import rospy
from sensor_msgs.msg import FluidPressure
from struct import pack, unpack

import mavros_msgs.msg  # importing only what you use is cleaner


class BaroSubscriber():
    def __init__(self):
        rospy.init_node('baro_subscriber', anonymous=True)
        self.subscriber = rospy.Subscriber('/mavlink/from', mavros_msgs.msg.Mavlink, self.listener_callback)

    def listener_callback(self, msg):
        print(msg)


def main():
    Baro = BaroSubscriber()
    rospy.spin()
    rospy.shutdown()


if __name__ == '__main__':
    main()
