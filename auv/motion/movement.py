import time
import rospy
from . import robot_control
from  ..utils import arm, disarm

rospy.init_node("Depth")

rc = robot_control.RobotControl()

arm.arm()
start_time = time.time()

rc.set_depth(0.7)
print("[INFO] Beginning test")

while time.time() - start_time < 15:
	rc.movement(lateral=1, forward=0, yaw=0)
	# Forward is actually a yaw (undetermined which way)
	# Lateral is
	# Yaw is 

rc.movement(lateral=0, forward=0, yaw=0)
rc.set_depth(0.0)

disarm.disarm()