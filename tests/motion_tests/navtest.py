import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("Nav Test", anonymous=True)
rc = robot_control.RobotControl()

arm.arm()
rc.set_absolute_depth(0.5)
time.sleep(5)
print("[INFO] This is the start")
rc.waypointNav(x=0,y=5) 


print("Reached the end")

disarm.disarm()
