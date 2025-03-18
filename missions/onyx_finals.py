"""
Running the gate mission (dead reckoning), style, buoy, and octagon missions.
Finals run for 2024 season. 
NOTE: This code did not give the intended result because the set_heading function only works when the differences between 
actual and desired heading are outside a certain range -- if these are too close together, the loop will stall (which prevented Onyx
from reaching the Octagon mission).
"""

import rospy
import time

from auv.mission import style_mission, buoy_mission, octagon_approach_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

rospy.init_node("Onyx", anonymous = True)

rc = robot_control.RobotControl(enable_dvl=False)

target = "Red"
gate_heading = 220

time.sleep(60)

arm.arm()

rc.set_depth(0.65)

time.sleep(5)
rc.set_heading(gate_heading)

curr_time = time.time()

while time.time() - curr_time < 25:
    rc.movement(forward=2)


# Run the style mission
style = style_mission.StyleMission()
style.run()
style.cleanup()

rc.set_heading(gate_heading + 35)
curr_time = time.time()
while time.time() - curr_time < 5:
    rc.movement(forward=2)

rc.set_heading(gate_heading - 70)

# Run the buoy mission
buoy = buoy_mission.BuoyMission(target)
buoy.run()
buoy.cleanup()

rc.set_depth(0.45)
rc.set_heading(gate_heading + 35)
# Get to the octagon, our model is short range only

curr_time = time.time()

while time.time() - curr_time < 15:
    rc.movement(forward=2.5)

rc.set_heading(gate_heading + 25)
curr_time = time.time()

while time.time() - curr_time < 50:
    rc.movement(lateral=-2.5)

# Octagon mission
octagon = octagon_approach_mission.OctagonApproachMission()
octagon.run()
octagon.cleanup()

time.sleep(1.0)

print("[INFO] Mission run terminate")
disarm.disarm()