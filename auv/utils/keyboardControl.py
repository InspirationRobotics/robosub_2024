"""
Keyboard control script for controlling our AUVs. This should be able to do everything that we need, i.e.
moving thrusters, firing servos, and using specialized motion functions in Robot Control like forward_dist
and set_depth.

NOTE: Keyboard library must be installed on the AUV's computer; you can install it using "pip install keyboard".
"""

import rospy
import keyboard
import time
import threading
from auv.motion.robot_control import RobotControl
from auv.motion.servo import Servo
from auv.utils import arm, disarm, deviceHelper

rospy.init_node("Control", anonymous=True)

rc = RobotControl()

current_sub = deviceHelper.variables.get("sub")
if current_sub == "onyx":
    servo = Servo()

forward = 0
lateral = 0
yaw = 0

manual_control = False
function_control = False
flag = True

arm.arm()

data_lock = threading.Lock()
SLEEP_INTERVAL = 0.05

def sendData():
    while flag:
        with data_lock:
            rc.movement(forward=forward, lateral=lateral, yaw=yaw, pitch=0, roll=0)
        time.sleep(SLEEP_INTERVAL)

thread_mov = threading.Thread(target=sendData)
thread_mov.daemon = True

while not manual_control and not function_control:
    settings = input("Type 'm' to start manual control, 'f' for function control: ")
    if settings == 'm':
        manual_control = True
    elif settings == 'f':
        function_control = True
    else:
        print("[Error] Invalid input.")

thread_mov.start()

try:
    while manual_control:
        event = keyboard.read_event(suppress=True)
        if event.event_type == keyboard.KEY_DOWN:
            with data_lock:
                var = event.name
                if var == "w":
                    forward = 1
                elif var == "s":
                    forward = -1
                elif var == "a":
                    lateral = -1
                elif var == "d":
                    lateral = 1
                elif var == "q":
                    yaw = -1
                elif var == "e":
                    yaw = 1
                elif var == "b":
                    forward = 0
                    lateral = 0
                    yaw = 0
                    print("[INFO] Aborting.")
                    manual_control = False
                    break
                else:
                    print("[WARN] Invalid input.")
        elif event.event_type == keyboard.KEY_UP:
            with data_lock:
                forward = 0
                lateral = 0
                yaw = 0
                print("[INFO] Idle.")

    while function_control:
        try:
            var = input()
            # Function control logic here
        except KeyboardInterrupt:
            function_control = False
            disarm.disarm()
            break

except KeyboardInterrupt:
    pass
finally:
    flag = False
    thread_mov.join()
    with data_lock:
        rc.movement(yaw=0, forward=0, lateral=0, pitch=0, roll=0)
    disarm.disarm()
    rospy.signal_shutdown("Manual shutdown.")
    print("[INFO] Control node shutdown.")

rospy.spin()