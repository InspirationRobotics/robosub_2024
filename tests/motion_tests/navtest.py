import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("NavTest", anonymous=True)
rc = robot_control.RobotControl()
rc.set_control_mode("depth_hold")
arm.arm()
rc.set_absolute_z(0.5)
time.sleep(5)
print("[INFO] This is the start")
rc.set_absolute_yaw(0.0)

print("Reached the end")

disarm.disarm()
