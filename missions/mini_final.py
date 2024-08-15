"""
To create a sequential order of missions for Onyx to follow.

Onyx will complete the Coin Flip, Gate, and Buoy missions.
"""

import rospy
import time

from auv.mission import style_mission, buoy_mission, octagon_approach_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

rospy.init_node("Onyx", anonymous = True)

rc = robot_control.RobotControl(enable_dvl=False)

gate_heading = 210

arm.arm()

rc.set_depth(0.45)

time.sleep(5)

# Rotate towards the heading of the gate, move 2 meters forward
rc.set_heading(gate_heading)

# Get to the octagon, our model is short range only

curr_time = time.time()

while time.time() - curr_time < 2:
    rc.movement(forward=2.5)


for i in range(3):
    rc.set_heading(gate_heading)
    curr_time = time.time()
    
    while time.time() - curr_time < 3:
        rc.movement(lateral=-2.5)

time.sleep(1.0)

print("[INFO] Mission run terminate")

disarm.disarm()
