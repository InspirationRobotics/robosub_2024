import rospy
import time

from . import robot_control
from auv.utils import arm, disarm

rospy.init_node("ThrusterTest", anonymous=True)
rc = robot_control.RobotControl()

arm.arm()

curr = time.time()

print("[DEBUG] Moving forward")

while time.time() - curr < 4:
    rc.movement(forward=2)


curr = time.time()

print("[DEBUG] Moving backward")

while time.time() - curr < 4:
    rc.movement(forward=-2)

disarm.disarm()