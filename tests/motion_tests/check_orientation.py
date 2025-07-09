import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("MotionTest", anonymous=True)
rc = robot_control.RobotControl(enable_dvl=False)


print("[INFO}This is the start")
rc.set_depth(0.9) 

time.sleep(5.0)


print("Reached the end")

rc.set_depth(0.0)
disarm.disarm()
