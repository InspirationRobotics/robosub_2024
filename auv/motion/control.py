import rospy
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
    if var == 'forward':
        forward = 2
    elif var == 'backward':
        forward = -2
    elif var == 'down':
        drop = 2
    elif var == 'up':
        drop = -2
    elif var == 'lateral right':
        lateral = 2 # Right
    elif var == 'lateral left':
        lateral = -2
    elif var == 'yaw cw':
        yaw = 2 # Clockwise
    elif var == 'yaw ccw':
        yaw = -2
    elif var == 'stop forward':
        forward = 0
    elif var == 'stop lateral':
        lateral = 0
    elif var == 'stop yaw':
        yaw = 0
    elif var == "stop drop":
        drop = 0
    elif var == 'i':
        forward = 0
        lateral = 0
        yaw  = 0
        depth = 0
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

except KeyboardInterrupt:
    channels = [1500] * 18
    msg.channels = channels
    command_pub.publish(msg)
    print("Exiting...")
    run = False

disarm()
