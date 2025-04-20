"""
To control the robot by setting PWM values to auv/devices/thrusters. The class RobotControl publishes PWM values to the MAVROS topics.
These MAVROS topics are predefined topics that the pixhawk subscribes to. RobotControl does not handle the interface between the 
pixhawk flight controller and the software -- that is the job that pixstandalone.py does. 
"""

import time

# Import the MAVROS message types that are needed
from geometry_msgs.msg import Twist
import mavros_msgs.msg
import mavros_msgs.srv
import rospy
import std_msgs
from std_msgs.msg import Float64, Float32MultiArray, String
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped


# Import the PID controller
from simple_pid import PID

# Get the mathematical functions that handle various navigation tasks from utils.py
from .utils import get_distance, get_heading_from_coords, heading_error, rotate_vector, inv_rotate_vector
from ..utils import deviceHelper # Get the configuration of the devices plugged into the sub(thrusters, camera, etc.)
from ..device.dvl import dvl # DVL class that enables position estimation
from ..device.fog import fog_interface as fog
import math
import numpy as np



class RobotControl:
    """
    Class to control the robot
    """

    def __init__(self, enable_dvl=True, enable_fog = False):
        """
        Initialize the RobotControl class

        Args:
            enable_dvl (bool): Flag to enable or disable DVL
        """

        # Initialize the configuration of the devices, depth of the sub, compass of the sub, DVL
        
        # Get the configuration of the devices plugged into the sub(thrusters, camera, etc.)
        self.config     = deviceHelper.variables            

        # Store informaiton
        self.mode             = ""
        self.pose:PoseStamped = None
        # Establish thruster and depth publishers
        # TODO add subscriber to get current pose and state
        self.sub_pose       = rospy.Subscriber("auv/status/pose", PoseStamped, self.set_depth)  
        self.sub_mode       = rospy.Subscriber("auv/status/mode", String, self.set_mode)
        self.pub_thrusters  = rospy.Publisher("auv/devices/thrusters", mavros_msgs.msg.OverrideRCIn, queue_size=10)
        self.pub_mode       = rospy.Publisher("auv/status/mode", String, queue_size=10)
        self.pub_button     = rospy.Publisher("/mavros/manual_control/send", mavros_msgs.msg.ManualControl, queue_size=10)

        # TODO: reset pix standalone depth Integration param 

        # A set of PIDs (Proportional - Integral - Derivative) to handle the movement of the sub
        """
        PIDs work by continously computing the error between the desired setpoint (desired yaw angle, forward velocity, etc.) and the 
        actual value. Based on this error, PIDs generate control signals to adjust the robot's actuators (in this case thrusters) to 
        minimize the desired setpoint. 

        Video: https://www.youtube.com/watch?v=wkfEZmsQqiA

        These definitions "tune" the PID controller for the necessities of the sub -- Proportional is tuned up high, which means greater 
        response to the current error but possible overshooting and oscillation
        """

        self.PIDs = {
            "yaw": PID(
                self.config.get("YAW_PID_P", 12),
                self.config.get("YAW_PID_I", 0.01),
                self.config.get("YAW_PID_D", 0.0),
                setpoint=0,
                output_limits=(-1, 1),
            ),
            "surge": PID(
                self.config.get("FORWARD_PID_P", 4.0),
                self.config.get("FORWARD_PID_I", 0.01),
                self.config.get("FORWARD_PID_D", 0.1),
                setpoint=0,
                output_limits=(-2, 2),
            ),
            "lateral": PID(
                self.config.get("LATERAL_PID_P", 4.0),
                self.config.get("LATERAL_PID_I", 0.01),
                self.config.get("LATERAL_PID_D", 0.1),
                setpoint=0,
                output_limits=(-2, 2),
            ), 
            "depth": PID(
                self.config.get("DEPTH_PID_P", 0.5),
                self.config.get("DEPTH_PID_I", 0.1),
                self.config.get("DEPTH_PID_D", 0.1),
                setpoint=0,
                output_limits=(-0.5, 0.5),
            ),  
            
        }

        # Wait for the topics to run
        time.sleep(1)

    def modeCallback(self, msg):
        """
        Callback function to handle the mode of the robot
        """
        self.mode = msg.data

    def setMode(self, msg):
        """
        Set the mode of the robot
        """
        self.mode = msg.data
        self.pub_mode.publish(String(msg.data))

    def movement(
        self,
        yaw=None,
        forward=None,
        lateral=None,
        pitch=None,
        roll=None,
        vertical=0,
        **kwargs,
    ):
        """
        Move the robot in a given direction, by directly changing the PWM value of each thruster. This does not take input from the DVL.
        This is a non-blocking function.
        Inputs are between -5 and 5

        Args:
            yaw (float): Power for the yaw maneuver
            forward (float): Power to move forward
            lateral (float): Power for moving laterally (negative one way (less than 1500), positive the other way (more than 1500))
            pitch (float): Power for the pitch maneuver
            roll (float): Power for the roll maneuver
            vertical (float): Distance to change the depth by

        # TODO Handle timeout of the pixhawk
        """
        # swtich to manual/stablize mode if not in alt hold mode
        if self.mode != "ALT_HOLD":
            self.pub_mode.publish(String("MANUAL"))  

        # Create a message to send to the thrusters
        pwm = mavros_msgs.msg.OverrideRCIn()

        # Calculate PWM values
        channels = [1500] * 18
        channels[0] = int((pitch * 80) + 1500) if pitch else 1500
        channels[1] = int((roll * 80) + 1500) if roll else 1500
        channels[2] = int((vertical * 80) + 1500) if vertical and self.mode != "ALT_HOLD" else 1500  
        channels[3] = int((yaw * 80) + 1500) if yaw else 1500
        channels[4] = int((forward * 80) + 1500) if forward else 1500
        channels[5] = int((lateral * 80) + 1500) if lateral else 1500
        pwm.channels = channels

        # Publish PWMs to /auv/devices/thrusters
        if vertical!=0: self.set_relative_depth(vertical)
        self.pub_thrusters.publish(pwm)

    def depth_hold(self):
        """
        Hold current depth
        """
        self.pub_mode.publish(String("ALTCTL"))

    def set_absolute_depth(self, depth):
        """
        Set the depth of the robot

        Args:
            depth (float): Depth to set the robot to
        """
        # Clear the PID error
        self.PIDs["depth"].clear()

        # Get the current depth
        if self.pose is None:
            rospy.logwarn("Pose is not set")
            return

        # Change mode to stablize
        self.setMode("STABILIZE")

        # Get the current depth
        current_depth = self.pose.pose.position.z

        # Calculate the error
        error = depth - current_depth

        # Calculate the currect pwm
        pwm = int(self.PIDs["depth"].setpoint(error))

        # Set thruster pwm
        self.movement(vertical=pwm)

    def set_relative_deph(self, depth):
        """
        Set the depth of the robot relative to the current depth

        Args:
            depth (float): Relative depth to set the robot to (-2 means up, 2 means down)
        """
        # Clear the PID error
        self.PIDs["depth"].clear()
        
        # Get the current depth
        if self.pose is None:
            rospy.logwarn("Pose is not set")
            return

        # Change mode to stablize
        self.setMode("STABILIZE")

        # Calculate the currect pwm
        pwm = int(self.PIDs["depth"].setpoint(depth))

        # Set thruster pwm
        self.movement()




    
