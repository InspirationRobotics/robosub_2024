import rospy
from sensor_msgs.msg import FluidPressure
from struct import pack, unpack
import std_msgs
import mavros_msgs.msg  # importing only what you use is cleaner


class CompassSubscriber():
    def __init__(self):
        rospy.init_node('compass_subscriber', anonymous=True)
        print("CompassSubscriber initialized")
        self.subscriber = rospy.Subscriber('/mavros/global_position/compass_hdg', std_msgs.msg.Float64, self.listener_callback)
        print("CompassSubscriber subscriber initialized")

    def listener_callback(self, msg):
        print("CompassSubscriber listener_callback called")
        print(msg)


def main():
    Baro = CompassSubscriber()
    rospy.spin()



if __name__ == '__main__':
    main()
