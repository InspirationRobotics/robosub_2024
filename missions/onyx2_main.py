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

rc.set_depth(0.5)

time.sleep(5)

# Rotate towards the heading of the gate, move 2 meters forward
rc.set_heading(gate_heading)

# Run the gate mission using just the DVL lol

rc.lateral_dvl(distance=-0.7)
rc.forward_dvl(distance=7)


# Run the style mission
style = style_mission.StyleMission()
style.run()
style.cleanup()
rc.set_heading(gate_heading)

# Run the buoy mission
buoy = buoy_mission.BuoyMission(target)
buoy.run()
buoy.cleanup()



rc.set_heading(gate_heading)

# Get to the octagon, our model is short range only
rc.forward_dvl(distance=10)


# Octagon mission
octagon = octagon_approach_mission.OctagonApproachMission()
octagon.run()
octagon.cleanup()

time.sleep(1.0)

print("[INFO] Mission run terminate")

disarm.disarm()
