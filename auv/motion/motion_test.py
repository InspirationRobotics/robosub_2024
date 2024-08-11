import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("MotionTest", anonymous=True)
rc = robot_control.RobotControl(enable_dvl=False)

arm.arm()
time.sleep(3.0)

rc.set_depth(0.5)
# rc.set_mode("MANUAL")
#first_time = time.time()


first_time = time.time()
while time.time() - first_time < 3:
    rc.movement(forward = 2)

first_time = time.time()
while time.time() - first_time < 3:
    rc.movement(forward = -2)

#rc.set_relative_depth(0.1)

#time.sleep(5)

#rc.set_relative_depth(-0.1)

time.sleep(5)
# rc.button_press(256)

first_time = time.time()
while time.time() - first_time < 3:
    rc.movement(lateral = 2)

first_time = time.time()
while time.time() - first_time < 3:
    rc.movement(lateral = -2)

first_time = time.time()
while time.time() - first_time < 3:
    rc.movement(yaw = 2)

first_time = time.time()
while time.time() - first_time < 3:
    rc.movement(yaw = -2)

time.sleep(1.0)

disarm.disarm()
