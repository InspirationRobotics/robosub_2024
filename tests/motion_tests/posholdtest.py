import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm

rospy.init_node("poshold_test", anonymous=True)  # avoid hiearchy issue
rc = robot_control.RobotControl(debug=True)

arm.arm()
time.sleep(3.0)
print("[INFO}This is the start")
rc.set_depth(0.9)

time.sleep(20)
print("[INFO] Reached the end of the program")

disarm.disarm()
