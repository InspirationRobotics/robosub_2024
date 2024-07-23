import rospy
import time

from . import robot_control
from auv.utils import arm, disarm

rospy.init_node("ThrusterTest", anonymous=True)
rc = robot_control.RobotControl()

arm.arm()

curr = time.time()

while time.time() - curr < 0.01:
    rc.movement(forward=2)

curr = time.time()

while time.time() - curr < 0.01:
    rc.movement(forward=-2)

disarm.disarm()