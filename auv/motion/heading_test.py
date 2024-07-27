import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("HeadingTest", anonymous=True)
rc = robot_control.RobotControl()


# rc.get_callback_compass()

#arm.arm()

time.sleep(5)

heading = rc.get_heading()
print(f"[DEBUG]: Heading is {heading}")
#rc.set_heading(heading + 180)

time.sleep(2.0)

#disarm.disarm()
