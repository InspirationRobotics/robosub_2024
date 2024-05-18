import rospy
import time
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
from . import robot_control

rospy.init_node("Control")

rate = rospy.Rate(10)
command_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size = 10)
arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

run = True

forward = 0
lateral = 0
yaw = 0
drop = 0

rc = robot_control.RobotControl()

# TODO: Check the lateral, yaw and make sure that we can input it both ways.
# TODO: Create a function that controls absolute depth, as well as simple upwards and downwards.

def control():
    global forward, lateral, yaw, drop, run
    var = input("Type in a command: ")
    if var == 'f':
        forward = 1.5
    elif var == 'b':
        forward = -1.5
    elif var == 'd':
        drop = 1
    elif var == 'u':
        drop = -1
    elif var == 'lr':
        lateral = 1.5 # Right
    elif var == 'll':
        lateral = -1.5
    elif var == 'y cw':
        yaw = 0.5 # Clockwise
    elif var == 'y ccw':
        yaw = -0.5
    elif var == 'sf':
        forward = 0
    elif var == 'sl':
        lateral = 0
    elif var == 'sy':
        yaw = 0
    elif var == "sd":
        drop = 0
    elif var == 'set depth':
        drop = 0
        drop_control = input("Starting power: ")
        start_time = time.time()
        valid_input = is_convertible_to_float(drop_control)
        if valid_input == True:
            drop += float(drop_control)
        else:
            print("Invalid input.")
        while True:
            stop_input = input("Key in 'stop' to keep the depth of the sub: ")
            if stop_input == "stop":
                break
            elif time.time() - start_time == 0.5:
                start_time = time.time()
                drop += 0.2
                print(f'{drop}')
                
        # depth = input("Absolute depth: ")
        # rc.set_depth(depth)
    elif var == 'i':
        forward = 0
        lateral = 0
        yaw  = 0
        drop = 0
    elif var == 's':
        forward = 0
        lateral = 0
        yaw = 0
        run = False
        disarm()
    else:
        print("Invalid command.")

def arm():
    """Arm autonomous function"""
    try: 
        arm_service(True)
        rospy.loginfo("Vehicle armed.")
        return True
    
    except rospy.ServiceException as e:
        rospy.logwarn("Failed to arm vehicle: %s" % e)

def disarm():
    """Disarm autonomous function"""
    try:
        arm_service(False)
        rospy.loginfo("Vehicle disarmed.")
        return True
    except rospy.ServiceException as e:
        rospy.logwarn("Failed to disarm vehicle: %s" % e)

def is_convertible_to_float(value):
    try:
        float(value)
        return True
    except ValueError:
        return False

arm()

try:
    while run:
        control()
        msg = OverrideRCIn()
        channels = [1500] * 18
        channels[2] = int((drop * 80) + 1500)
        channels[3] = int((yaw * 80)) + 1500
        channels[4] = int((forward * 80) + 1500)
        channels[5] = int((lateral * 80) + 1500)
        msg.channels = channels
        command_pub.publish(msg)
        print(f"Forward is {forward}")
        print(f"Lateral is {lateral}")
        print(f"Yaw is {yaw}")
        print(f"Drop power is {drop}")

except KeyboardInterrupt:
    channels = [1500] * 18
    msg.channels = channels
    command_pub.publish(msg)
    print("Exiting...")
    run = False

disarm()