import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("MotionTest", anonymous=True)
rc = robot_control.RobotControl(enable_dvl=False)

arm.arm()
time.sleep(3.0)

rc.set_depth(0.8)
# rc.set_mode("MANUAL")
#first_time = time.time()
time.sleep(5.0)

first_time = time.time()
while time.time() - first_time < 21:
   rc.movement(forward = 2)

first_time = time.time()
while time.time() - first_time < 4.5:
    rc.movement(lateral = -2)

first_time = time.time()
while time.time() - first_time < 18:
   rc.movement(forward = -2)


#current_heading = rc.get_heading("vectornav_imu")
#print(current_heading)
#rc.set_heading(current_heading + 90, "vectornav_imu")



#first_time = time.time()
#while time.time() - first_time < 1:
    #rc.movement(forward = -2)

#rc.set_relative_depth(0.1)

#time.sleep(5)

#rc.set_relative_depth(-0.1)

#time.sleep(5)
# rc.button_press(256)

#first_time = time.time()
#while time.time() - first_time < 3:
    #rc.movement(lateral = -2)

#first_time = time.time()
#while time.time() - first_time < 1:
    #rc.movement(lateral = -2)

# first_time = time.time()
# while time.time() - first_time < 1.1:
#     rc.movement(yaw = 2)

#first_time = time.time()
#while time.time() - first_time < 2:
     #rc.movement(yaw = -2)

#first_time = time.time()
#while time.time() - first_time < 3:
     #rc.movement(forward = 2)

#first_time = time.time()
#while time.time() - first_time < 2:
     #rc.movement(forward = 2)

#first_time = time.time()
#while time.time() - first_time < 15:
    #rc.movement(forward = 2)

#time.sleep(1.0)
rc.set_depth(0.0)
disarm.disarm()

