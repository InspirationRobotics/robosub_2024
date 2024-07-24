import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("HeadingTest", anonymous=True)
rc = robot_control.RobotControl()


# rc.get_callback_compass()

arm.arm()

time.sleep(5)

rc.forward_dvl(throttle=1, distance=4)

time.sleep(1)

disarm.disarm()
