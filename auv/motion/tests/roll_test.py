import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("roll_test", anonymous=True)
rc = robot_control.RobotControl(enable_dvl=False)

arm.arm()
time.sleep(3.0)

first_time = time.time()
while time.time() - first_time < 10:
    rc.movement(pitch=200)

# first_time = time.time()
# while time.time() - first_time < 3:
#     rc.movement(forward = -2)

# first_time = time.time()
# while time.time() - first_time < 3:
#     rc.movement(lateral = 2)

# first_time = time.time()
# while time.time() - first_time < 3:
#     rc.movement(lateral = -2)

time.sleep(1.0)

disarm.disarm()
