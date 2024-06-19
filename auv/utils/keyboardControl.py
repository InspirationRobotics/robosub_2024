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

from ..motion.robot_control import RobotControl
from ..motion.servo import Servo
from . import arm, disarm, deviceHelper

rospy.init_node("Control", anonymous = True)

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

def sendData():
    while flag:
        rc.movement(forward=forward, lateral=lateral, yaw=yaw, pitch=0, roll=0)
        time.sleep(0.05)
    
thread_mov = threading.Thread(target=sendData)
thread_mov.daemon = True
thread_mov.start()

while not manual_control and not function_control:
    settings = input("Type 'm' to start manual control, 'f' for function control.")
    if settings == 'm':
        manual_control = True
    elif settings == 'f':
        function_control = True
    else:
        print("[Error] Invalid input.")

while manual_control:
    try:
        event = keyboard.read_event(suppress=True)  # Read key event without echoing to console

        if event.event_type == keyboard.KEY_DOWN:
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
                lateral = 0
                forward = 0
                yaw = 0
                print("[INFO] Aborting.")
                manual_control = False
                break
            else:
                print("[WARN] Invalid input.")
        
        else:
            forward = 0
            lateral = 0
            yaw = 0
            print("[INFO] Idle.")

    except KeyboardInterrupt:
        flag = False
        disarm.disarm()
        break

    while function_control:
        try:
            var = input()
            pass
        except:
            function_control = False
            disarm.disarm()
            break

flag = False
thread_mov.join()

rc.movement(yaw = 0, forward = 0, lateral = 0, pitch = 0, roll = 0)
disarm.disarm()