import time

from auv.mission import octagon_approach_mission
from auv.utils import arm, disarm, deviceHelper
from auv.motion import robot_control

import rospy
rospy.init_node("missions", anonymous=True)

rc = robot_control.RobotControl()

# load sub config
config = deviceHelper.variables
arm.arm()

rc.set_depth(0.38)

time.sleep(5.0)

# create the mission object
octagonMission = octagon_approach_mission.OctagonApproachMission(**config)

# run the mission
surfacingMission.run()

# terminate the mission
surfacingMission.cleanup()

# end
print("[INFO] Octagon Mission ended")
disarm.disarm()
