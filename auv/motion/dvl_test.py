import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("DVLTest", anonymous=True)
rc = robot_control.RobotControl()


# rc.get_callback_compass()

arm.arm()

time.sleep(5)

rc.forward_dvl(distance = -1)

time.sleep(2.0)

disarm.disarm()
