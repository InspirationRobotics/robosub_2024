import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("HeadingTest", anonymous=True)
rc = robot_control.RobotControl()


# rc.get_callback_compass()

arm.arm()

time.sleep(5)

first_time  = time.time()
while time.time() - first_time < 4.0:
    rc.movement(yaw = -2)

time.sleep(2.0)

disarm.disarm()
