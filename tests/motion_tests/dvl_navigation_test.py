import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm

rospy.init_node("DVLNavigationTest", anonymous=True)
rc = robot_control.RobotControl()
arm.arm()
time.sleep(3.0)
print("[INFO] Starting DVL navigation test")

rc.set_depth(0.2)
rc.set_heading(90, "vectornav_imu")
time.sleep(2.0)
rc.navigate_dvl(x=0, y=5, z=0, end_heading=90, update_freq=10)
time.sleep(2.0)
rc.navigate_dvl(x=2.5, y=0, z=0, end_heading=90, update_freq=10)
time.sleep(2.0)
rc.navigate_dvl(x=0, y=0, z=0.5, end_heading=180, update_freq=10)
disarm.disarm()
print("[INFO] DVL navigation test completed")

