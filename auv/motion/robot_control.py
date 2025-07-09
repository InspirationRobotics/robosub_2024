"""
To control the robot by setting PWM values to auv/devices/thrusters. The class RobotControl publishes PWM values to the MAVROS topics.
These MAVROS topics are predefined topics that the pixhawk subscribes to. RobotControl does not handle the interface between the 
pixhawk flight controller and the software -- that is the job that pixstandalone.py does. 
"""


# Import the MAVROS message types that are needed
import rospy
import std_msgs
from std_msgs.msg import Float64, Float32MultiArray, String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import mavros_msgs.msg
import mavros_msgs.srv
import geometry_msgs.msg



# Get the mathematical functions that handle various navigation tasks from utils.py
from auv.utils import deviceHelper # Get the configuration of the devices plugged into the sub(thrusters, camera, etc.)
from auv.utils import arm, disarm
from auv.device.dvl import dvl # DVL class that enables position estimation
from auv.device.fog import fog_interface as fog

from simple_pid import PID
from transforms3d.euler import euler2quat
from transforms3d.euler import quat2euler
import threading
import numpy as np
import time
import math




class RobotControl:
    """
    Class to control the robot
    """

    def __init__(self, debug=False):
        """
        Initialize the RobotControl class

        Args:
            enable_dvl (bool): Flag to enable or disable DVL
        """
        self.rate = rospy.Rate(10) # 10 Hz
        # Get the configuration of the devices plugged into the sub(thrusters, camera, etc.)
        self.config     = deviceHelper.variables
        self.debug      = debug            

        # Store informaiton
        self.mode           = ""
        self.position       = {'x':0,'y':0,'z':0}
        self.orientation    = {'yaw':0,'pitch':0,'roll':0}

        # Establish thruster and depth publishers
        self.sub_pose       = rospy.Subscriber("auv/status/pose", PoseStamped, self.pose_callback)  
        self.pub_thrusters  = rospy.Publisher("auv/devices/thrusters", mavros_msgs.msg.OverrideRCIn, queue_size=10)
        self.pub_button     = rospy.Publisher("/mavros/manual_control/send", mavros_msgs.msg.ManualControl, queue_size=10)

        # store desire point
        self.desired_point  = {"x":None,"y":None,"z":None,"heading":None}
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

        # Arm the robot
        arm.arm()

        # Run thread
        self.thread = threading.Thread(target=self.publisherThread)
        self.thread.daemon = True
        self.thread.start()

    def pose_callback(self, msg):
        self.position['x'] = msg.pose.position.x
        self.position['y'] = msg.pose.position.y
        self.position['z'] = msg.pose.position.z

        roll, pitch, yaw = quat2euler([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
        self.orientation['yaw']     = yaw
        self.orientation['pitch']   = pitch
        self.orientation['roll']    = roll

    def publisherThread(self):
        """
        Publisher to publish the thruster values
        """
        while not rospy.is_shutdown():
            # Get desire x,y,z
            x       = self.desired_point["x"] if self.desired_point["x"] is not None else self.position['x']
            y       = self.desired_point["y"] if self.desired_point["y"] is not None else self.position['y']
            z       = self.desired_point["z"] if self.desired_point["z"] is not None else self.position['z']
            heading = self.desired_point["heading"] if self.desired_point["heading"] is not None else self.orientation['yaw']
            # Calculate error
            x_error     = x - self.position['x']
            y_error     = y - self.position['y']
            z_error     = z - self.position['z']
            yaw_error   = heading - self.orientation['yaw']

            # Get the PWM values
            lateral_pwm = self.PIDs["lateral"](x_error)
            surge_pwm   = self.PIDs["surge"](y_error)
            depth_pwm   = self.PIDs["depth"](z_error)
            yaw_pwm     = self.PIDs["yaw"](yaw_error)

            # Set the PWM values
            self.movement(lateral=lateral_pwm, forward=surge_pwm, vertical=depth_pwm, yaw=yaw_pwm)
            
            time.sleep(0.1) # 10hz

    def movement(
        self,
        yaw=None,
        forward=None,
        lateral=None,
        pitch=None,
        roll=None,
        vertical=None,
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
        """
        # Create a message to send to the thrusters
        pwm = mavros_msgs.msg.OverrideRCIn()

        # Calculate PWM values
        channels = [1500] * 18
        channels[0] = int((pitch * 80) + 1500) if pitch else 1500
        channels[1] = int((roll * 80) + 1500) if roll else 1500
        channels[2] = int((vertical * 80) + 1500) if vertical else 1500  
        channels[3] = int((yaw * 80) + 1500) if yaw else 1500
        channels[4] = int((forward * 80) + 1500) if forward else 1500
        channels[5] = int((lateral * 80) + 1500) if lateral else 1500
        pwm.channels = channels

        # Publish PWMs to /auv/devices/thrusters
        if not self.debug:
            self.pub_thrusters.publish(pwm)
        else:
            rospy.logdebug(f"pwms : {channels[0:6]}")

    def set_control_mode(self, msg:String):
        """
        Callback function to handle the control mode of the robot.

        Args:
            msg (String): The control mode command. Expected values are:
                        - "pid" for PID control
                        - "direct" for direct thruster control
        """
        self.mode = msg
        if self.mode=="direct":
            self.movement() # set 0s on all channel
            self.thread.join()
            for key, pid in self.PIDs.items():
                pid.reset()
        elif self.mode=="pid":
            for key, pid in self.PIDs.items():
                pid.reset()
            self.thread = threading.Thread(target=self.publisherThread)
            self.thread.daemon = True
            self.thread.start()
        else:
            rospy.logewarn("Control mode not found")
        
    def set_absolute_z(self, depth):
        """
        Set the depth of the robot

        Args:
            depth (float): Depth to set the robot to
        """
        # Clear the PID error
        self.PIDs["depth"].reset()
        self.desired_point["z"] = depth
    
    def set_absolute_x(self, x):
        """
        Set the x position of the robot

        Args:
            x (float): X position to set the robot to
        """
        # Clear the PID error
        self.PIDs["lateral"].reset()
        self.desired_point["x"] = x

    def set_absolute_y(self, y):
        """
        Set the y position of the robot

        Args:
            y (float): Y position to set the robot to
        """
        # Clear the PID error
        self.PIDs["surge"].reset()
        self.desired_point["y"] = y

    def set_absolute_heading(self, heading):
        """
        Set the heading of the robot

        Args:
            heading (float): Heading to set the robot to
        """
        # Clear the PID error
        self.PIDs["yaw"].reset()
        self.desired_point["heading"] = heading
    
    def set_relative_z(self, depth):
        """
        Set the depth of the robot relative to the current depth

        Args:
            depth (float): Relative depth to set the robot to (-2 means up, 2 means down)
        """
        # Clear the PID error
        self.PIDs["depth"].reset()
        self.desired_point["z"] = depth + self.pose.pose.position.z

    def set_relative_x(self, x):
        """
        Set the x position of the robot relative to the current position

        Args:
            x (float): Relative x position to set the robot to (-2 means left, 2 means right)
        """
        # Clear the PID error
        self.PIDs["lateral"].reset()
        self.desired_point["x"] = x + self.pose.pose.position.x

    def set_relative_y(self, y):
        """
        Set the y position of the robot relative to the current position

        Args:
            y (float): Relative y position to set the robot to (-2 means left, 2 means right)
        """
        # Clear the PID error
        self.PIDs["surge"].reset()
        self.desired_point["y"] = y + self.pose.pose.position.y

    def set_relative_heading(self, heading):
        """
        Set the heading of the robot relative to the current heading

        Args:
            heading (float): Relative heading to set the robot to (-2 means left, 2 means right)
        """
        # Clear the PID error
        self.PIDs["yaw"].reset()
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

    
    




    