"""
We made this script just to move Onyx to the other side of the pool
after a test fails - nothing to see here
"""

import rospy
import time

from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

rospy.init_node("prequal_mission", anonymous = True)

rc = robot_control.RobotControl()
movement_list = [-2, 4, 1] # lateral, forward, yaw


arm.arm()
time.sleep(5)

first_time = time.time()

while time.time() - first_time < 10:
    rc.movement(forward = movement_list[1])

time.sleep(1)

disarm.disarm()