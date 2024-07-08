import time
import rospy
from . import robot_control
from  ..utils import arm, disarm

rospy.init_node("Depth")

rc = robot_control.RobotControl()

arm.arm()
start_time = time.time()

while time.time() - start_time < 20:
	rc.set_depth(0.8)

disarm.disarm()
