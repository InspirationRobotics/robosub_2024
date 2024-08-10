import rospy 
import time

from auv.motion import robot_control
from auv.mission import gate_mission
from auv.utils import arm, disarm, deviceHelper

rospy.init_node("GATE_MISSION", anonymous=True)

rc = robot_control.RobotControl()

config = deviceHelper.variables
time.sleep(60)
arm.arm()

rc.set_depth(0.65)
time.sleep(5.0)

gate_heading = 220
rc.set_heading(gate_heading)

gateMission = gate_mission.GateMission(**config)
gateMission.run()
gateMission.cleanup()

print("[INFO] Gate Mission terminated.")
disarm.disarm()
