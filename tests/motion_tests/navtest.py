import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("Nav Test", anonymous=True)
rc = robot_control.RobotControl()
rc.set_control_mode("depth_hold")
arm.arm()
rc.set_absolute_z(0.5)
time.sleep(5)
print("[INFO] This is the start")
rc.waypointNav(x=1,y=5) 
time.sleep(10)


print("Reached the end")

disarm.disarm()
