"""
Template file to call individual missions/run them in a sequence.
"""

import rospy
import time

from auv.mission import style_mission, buoy_mission, octagon_approach_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

rospy.init_node("Onyx", anonymous = True)
initial_heading = 218

rc = robot_control.RobotControl(enable_dvl=False)
time.sleep(60) # Assumes that we are using tether, so there needs to be time for the operator to remove the tether/put in the connection plug.
arm.arm()
rc.set_depth(0.65)
time.sleep(5)

rc.set_heading(initial_heading)
curr_time = time.time()

# Move forward for 25 seconds.
while time.time() - curr_time < 25:
    rc.movement(forward=2)

# Run the style mission
style = style_mission.StyleMission()
style.run()
style.cleanup()

rc.set_heading(initial_heading + 35)

# Run the buoy mission
buoy = buoy_mission.BuoyMission()
buoy.run()
buoy.cleanup()

print("[INFO] Mission run terminate")
disarm.disarm()