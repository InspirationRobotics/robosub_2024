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
import threading
from auv.utils import arm, disarm

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

        # Initialize the node
        rospy.init_node("robot_control", anonymous=True)
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

        # Arm the robot
        arm.arm()
        # store desire point
        self.desired_point = {"x":None,"y":None,"z":None,"heading":None}
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
            ),  
            
        }

        # Wait for the topics to run
        time.sleep(1)

        # Run thread
        self.thread = threading.Thread(target=self.publisherThread)
        self.thread.daemon = True
        self.thread.start()

    def publisherThread(self):
        """
        Publisher to publish the thruster values
        """
        while not rospy.is_shutdown():
            # Publish the thruster values
            self.pub_thrusters.publish(self.pwm)
            # Get desire x,y,z
            x = self.desired_point["x"] if self.desired_point["x"] is not None else self.pose.pose.position.x
            y = self.desired_point["y"] if self.desired_point["y"] is not None else self.pose.pose.position.y
            z = self.desired_point["z"] if self.desired_point["z"] is not None else self.pose.pose.position.z
            heading = self.desired_point["heading"] if self.desired_point["heading"] is not None else self.pose.pose.orientation.z
            # Calculate error
            x_error = x - self.pose.pose.position.x
            y_error = y - self.pose.pose.position.y
            z_error = z - self.pose.pose.position.z
            heading_error = heading - self.pose.pose.orientation.z

            # Calculate PWM needed using pid
            self.PIDs["lateral"].setpoint = y
            self.PIDs["surge"].setpoint = x
            self.PIDs["depth"].setpoint = z
            self.PIDs["yaw"].setpoint = heading

            # Get the PWM values
            lateral_pwm = self.PIDs["lateral"](self.pose.pose.position.y)
            surge_pwm = self.PIDs["surge"](self.pose.pose.position.x)
            depth_pwm = self.PIDs["depth"](self.pose.pose.position.z)
            yaw_pwm = self.PIDs["yaw"](self.pose.pose.orientation.z)
            # Set the PWM values
            self.movement(lateral=lateral_pwm, forward=surge_pwm, vertical=depth_pwm, yaw=yaw_pwm)
            
            time.sleep(0.1) # 10hz

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

        self.pub_mode.publish(String("STASILIZE"))  

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
        self.pub_thrusters.publish(pwm)

    def set_absolute_z(self, depth):
        """
        Set the depth of the robot

        Args:
            depth (float): Depth to set the robot to
        """
        # Clear the PID error
        self.PIDs["depth"].clear()
        self.desired_point["z"] = depth
    
    def set_absolute_x(self, x):
        """
        Set the x position of the robot

        Args:
            x (float): X position to set the robot to
        """
        # Clear the PID error
        self.PIDs["lateral"].clear()
        self.desired_point["x"] = x

    def set_absolute_y(self, y):
        """
        Set the y position of the robot

        Args:
            y (float): Y position to set the robot to
        """
        # Clear the PID error
        self.PIDs["surge"].clear()
        self.desired_point["y"] = y

    def set_absolute_heading(self, heading):
        """
        Set the heading of the robot

        Args:
            heading (float): Heading to set the robot to
        """
        # Clear the PID error
        self.PIDs["yaw"].clear()
        self.desired_point["heading"] = heading
    def set_relative_z(self, depth):
        """
        Set the depth of the robot relative to the current depth

        Args:
            depth (float): Relative depth to set the robot to (-2 means up, 2 means down)
        """
        # Clear the PID error
        self.PIDs["depth"].clear()
        self.desired_point["z"] = depth + self.pose.pose.position.z

    def set_relative_x(self, x):
        """
        Set the x position of the robot relative to the current position

        Args:
            x (float): Relative x position to set the robot to (-2 means left, 2 means right)
        """
        # Clear the PID error
        self.PIDs["lateral"].clear()
        self.desired_point["x"] = x + self.pose.pose.position.x

    def set_relative_y(self, y):
        """
        Set the y position of the robot relative to the current position

        Args:
            y (float): Relative y position to set the robot to (-2 means left, 2 means right)
        """
        # Clear the PID error
        self.PIDs["surge"].clear()
        self.desired_point["y"] = y + self.pose.pose.position.y

    def set_relative_heading(self, heading):
        """
        Set the heading of the robot relative to the current heading

        Args:
            heading (float): Relative heading to set the robot to (-2 means left, 2 means right)
        """
        # Clear the PID error
        self.PIDs["yaw"].clear()
        self.desired_point["heading"] = heading + self.pose.pose.orientation.z

    def exit(self):
        """
        Exit the robot control
        """
        # Stop the robot
        self.movement()
        # Stop the thread
        self.thread.join()
        # Stop the node
        disarm.disarm()
        # Exit the node
        rospy.signal_shutdown("Exiting robot control")

    
    




    
