import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("HeadingTest", anonymous=True)
rc = robot_control.RobotControl()


# rc.get_callback_compass()

arm.arm()

time.sleep(5)

rc.get_callback_compass()
rc.set_heading(1)

time.sleep(2.0)

disarm.disarm()
