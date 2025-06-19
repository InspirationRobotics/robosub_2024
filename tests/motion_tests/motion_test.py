import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("MotionTest", anonymous=True)
rc = robot_control.RobotControl(enable_dvl=False)

#time.sleep(60.0)
#initial_time = time.time()
#while( initial_time - time.time < 57.0):
#   print(initial_time - time.time())
#print("Starting Now...")
    
arm.arm()
time.sleep(3.0)
print("This is the start")
rc.set_depth(0.8)
# rc.set_mode("MANUAL")
#first_time = time.time()
time.sleep(5.0)

#current_heading = rc.get_heading()
#target_heading = current_heading + 90
#rc.set_heading(90, "vectornav_imu")

#first_time = time.time()
#while time.time() - first_time < 26:
 #  rc.movement(forward = 2)

first_time = time.time()
while time.time() - first_time < 4.5:
    rc.movement(lateral = -2)

#first_time = time.time()
#while time.time() - first_time < 6:
   # rc.movement(forward =-2)

#first_time = time.time()
#while time.time() - first_time < 5:
 #   rc.movement(lateral = 2)

#first_time = time.time()
#while time.time() - first_time < 16:
 #  rc.movement(forward = -2)


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
print("Reached the end")
rc.set_depth(0.0)
disarm.disarm()

