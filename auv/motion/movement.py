import time
import rospy
from . import robot_control
from  ..utils import arm, disarm

rospy.init_node("Depth")

rc = robot_control.RobotControl()

arm.arm()
start_time = time.time()

while time.time() - start_time < 15:
	rc.movement(lateral=0, forward=-1, yaw=0)

disarm.disarm()
