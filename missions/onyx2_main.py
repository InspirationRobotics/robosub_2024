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

rc = robot_control.RobotControl()

target = "Red"
gate_heading = 220

arm.arm()

rc.set_depth(0.65)

time.sleep(5)

# Rotate towards the heading of the gate, move 2 meters forward
rc.set_heading(gate_heading)

# Run the gate mission using just the DVL lol

curr_time = time.time()

while time.time() - curr_time < 22:
    rc.movement(forward=2)


# Run the style mission
style = style_mission.StyleMission()
style.run()
style.cleanup()
# rc.set_heading(gate_heading - 45)

# # curr_time = time.time()
# # while time.time() - curr_time < 7:
#     rc.movement(forward=2)

# Run the buoy mission
buoy = buoy_mission.BuoyMission(target)
buoy.run()
buoy.cleanup()
rc.set_heading(260)
# Get to the octagon, our model is short range only

curr_time = time.time()

while time.time() - curr_time < 20:
    rc.movement(forward=2.5)

# Octagon mission
octagon = octagon_approach_mission.OctagonApproachMission()
octagon.run()
octagon.cleanup()

time.sleep(1.0)

print("[INFO] Mission run terminate")

disarm.disarm()