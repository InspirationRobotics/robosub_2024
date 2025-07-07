"""
Handles the MAVROS link between Pixhawk and software
"""

# To obtain information about the Linux distribution
import lsb_release 

if lsb_release.get_distro_information()["RELEASE"] == "18.04":
    import ctypes

    libgcc_s = ctypes.CDLL("libgcc_s.so.1")

"""
Importing necessary modules for platform information, signal handling, threading,
time, statistical analysis, converting between python values and C structs (struct)
"""
import platform
import signal
import threading
import time
from statistics import mean
from struct import pack, unpack

# Importing various message types for ROS
import geographic_msgs.msg
import geometry_msgs.msg
import mavros_msgs.msg
import mavros_msgs.srv
import sensor_msgs.msg
import std_msgs.msg

# Importing ROS, Numpy, PID controller
import numpy as np
import rospy
from simple_pid import PID

# For turning a status LED (based on a state) on and getting the configuration of a specified device, respectively
from ..utils import statusLed, deviceHelper

# For handling ROS topics
from ..utils.rospyHandler import RosHandler
from ..utils.topicService import TopicService

# Different modes/states of travel (predefined modes used by the Pixhawk)
MODE_MANUAL     = "MANUAL"
MODE_STABILIZE  = "STABILIZE"
MODE_ALTHOLD    = "ALT_HOLD"
MODE_LOITER     = "LOITER"
MODE_AUTO       = "AUTO"
MODE_GUIDED     = "GUIDED"
MODE_ACRO       = "ACRO"

# Configuration of devices from either Graey or Onyx (depending)
config = deviceHelper.variables


class AUV(RosHandler):
    """
    Creates a class for the sub, inheriting from ROSHandler (all of the functions from RosHandler are now in AUV)
    """
    def __init__(self):        
        # Accessing the device configurations
        self.config = config

        # Initiating construction of the RosHandler superclass
        super().__init__()

        # Attributes relating to status
        self.armed  = False
        self.guided = False # feels unnecessary
        self.mode   = ""

        # Create neutral channels
        self.channels = [1500] * 18

        self.thrustTime = time.time()  # Reference timestamp for timeout for no new thruster commands
        
        # Depth status/values
        self.depth              = None
        self.do_hold_depth      = False
        self.depth_pwm          = 0

        # why do we have pid here?
        self.depth_sample_size  = 100 # Number of samples for calibraiton
        self.depth_samples      = []
        self.calibrate          = False # Whether to calibrate the depth or not
        self.depth_calib        = 0
        self.depth_pid_params   = config.get("depth_pid_params", [0.5, 0.1, 0.1]) # Get PID controller parameters from config, if not found use default values (second arg)
        self.depth_pid_offset   = config.get("depth_pid_offset", 1500) # Get PID offset from key, if not found set PWM value to default neutral (1500)
        
        # Initialize the depth PID controller
        self.depth_pid               = PID(*self.depth_pid_params, setpoint=0.5) # Go to depth at 0.5 m
        self.depth_pid.output_limits = (-self.depth_pid_params[0], self.depth_pid_params[0]) # Output limits of PID controller

        # Initialize topics from Pixhawk (through MAVROS)
        self.TOPIC_STATE        = TopicService("/mavros/state"      , mavros_msgs.msg.State)
        self.SERVICE_ARM        = TopicService("/mavros/cmd/arming" , mavros_msgs.srv.CommandBool)
        self.SERVICE_SET_MODE   = TopicService("/mavros/set_mode"   , mavros_msgs.srv.SetMode)
        self.SERVICE_SET_PARAM  = TopicService("/mavros/param/set"  , mavros_msgs.srv.ParamSet)    # I don't think it's useful
        self.SERVICE_GET_PARAM  = TopicService("/mavros/param/get"  , mavros_msgs.srv.ParamGet)    # Need more testing on this

        # Movement topics, only works/is applicable in autonomous mode
        self.TOPIC_SET_VELOCITY = TopicService("/mavros/setpoint_velocity/cmd_vel_unstamped", geometry_msgs.msg.Twist)  # being tested not working very well
        self.TOPIC_SET_RC_OVR   = TopicService("/mavros/rc/override", mavros_msgs.msg.OverrideRCIn)                     # Currently used for thruster control

        # Sensory data (IMU, compass, remote control input, input from flight controller, battery state)
        self.TOPIC_GET_IMU_DATA = TopicService("/mavros/imu/data", sensor_msgs.msg.Imu)
        self.TOPIC_GET_CMP_HDG  = TopicService("/mavros/global_position/compass_hdg", std_msgs.msg.Float64)
        self.TOPIC_GET_RC       = TopicService("/mavros/rc/in"  , mavros_msgs.msg.RCIn)
        self.TOPIC_GET_MAVBARO  = TopicService("/mavlink/from"  , mavros_msgs.msg.Mavlink)                              # This is the barometer
        # https://discuss.bluerobotics.com/t/ros-support-for-bluerov2/1550/24
        self.TOPIC_GET_BATTERY  = TopicService("/mavros/battery"    , sensor_msgs.msg.BatteryState)                     # Do we really have this?

        # Create custom ROS topics so other nodes can easily access the data
        self.AUV_COMPASS        = TopicService("/auv/devices/compass", std_msgs.msg.Float64)
        self.AUV_IMU            = TopicService("/auv/devices/imu", sensor_msgs.msg.Imu)
        self.AUV_BARO           = TopicService("/auv/devices/baro", std_msgs.msg.Float64)
        self.AUV_GET_THRUSTERS  = TopicService("/auv/devices/thrusters", mavros_msgs.msg.OverrideRCIn)
        self.AUV_GET_DEPTH      = TopicService("/auv/devices/setDepth", std_msgs.msg.Float64)
        self.AUV_GET_REL_DEPTH  = TopicService("/auv/devices/setRelativeDepth", std_msgs.msg.Float64)
        self.AUV_GET_ARM        = TopicService("/auv/status/arm", std_msgs.msg.Bool)
        self.AUV_GET_MODE       = TopicService("/auv/status/mode", std_msgs.msg.String)

        # initialize the ROS node
        rospy.init_node("info_node", anonymous=True)
        # rospy.Rate(60)   # Change rate to 10 if issues arise
        self.connected = True

        # Subscribe to topics
        self.enable_topics_for_read()

        # spin
        rospy.spin()

    def arm(self, status: bool):
        """
        Arms the sub

        Args:
            status (bool): Whether the sub will be armed or not

        Returns:
            result.success (bool): Whether the sub was able to be armed
            result.result (str): Containing extra information about the result of the arming operation
        """
        # Turning the red status LED on if sub is armed
        if status:
            statusLed.red(True)
        else:
            statusLed.red(False)
        # Creating a CommandBoolRequest message to arm/disarm the sub
        data = mavros_msgs.srv.CommandBoolRequest()
        data.value = status
        # Set the data of the arming topic
        self.SERVICE_ARM.set_data(data)

        # Call the ROS service to arm the sub
        result = self.service_caller(self.SERVICE_ARM, timeout=30)
        return result.success, result.result
   

    def batteryIndicator(self, msg):
        """To indicate low battery voltage by flashing the red status LED"""
        if self.config.get("battery_indicator", False):
            self.voltage = msg.voltage
            if self.voltage < 13.5:
                statusLed.flashRed()

    def enable_topics_for_read(self):
        """To subscribe to ROS topics"""
        self.topic_subscriber(self.TOPIC_STATE          , self.stateCallback)
        self.topic_subscriber(self.TOPIC_GET_IMU_DATA   , self.imuCallback)
        self.topic_subscriber(self.TOPIC_GET_CMP_HDG    , self.compassCallback)
        self.topic_subscriber(self.TOPIC_GET_RC)
        self.topic_subscriber(self.AUV_GET_THRUSTERS    , self.thrusterCallback)
        self.topic_subscriber(self.AUV_GET_ARM)
        self.topic_subscriber(self.AUV_GET_MODE         , self.changeModeCallback)
        self.topic_subscriber(self.TOPIC_GET_MAVBARO    , self.baroCallback)
        self.topic_subscriber(self.TOPIC_GET_BATTERY    , self.batteryIndicator)

    def stateCallback(self, data):
        """To update parameters (status of arming, mode) based on received data from ROS topics"""
        if self.connected:
            try:
                self.armed = data.armed
                if not self.armed:
                    self.depth_pid.reset()
                self.mode = data.mode
                self.guided = data.guided
            except Exception as e:
                print("state failed")
                print(e)

    def changeModeCallback(self, mode: str):
        """
        To change the mode of the sub

        Args:
            mode (str): The mode to change to
        
        Returns:
            result.mode_sent (str): The mode that was sent to autopilot to set the new mode
        """
        if not isinstance(mode, str):
            mode = str(mode)
            mode = mode[7:-1]

        print(f"[DEBUG] Set mode to {mode}")

        # Handle althold specially, setting mode to hold depth and to stabalize to be the new modes
        if mode == MODE_ALTHOLD:
            self.do_hold_depth = True
            mode = MODE_STABILIZE
        # Create a SetModeRequest message to change the mode
        data = mavros_msgs.srv.SetModeRequest()
        data.custom_mode = mode
        # Setting the data for the ROS service
        self.SERVICE_SET_MODE.set_data(data)
        # Call the ROS service to set the mode
        result = self.service_caller(self.SERVICE_SET_MODE, timeout=30)
        return result.mode_sent
    
    def thrusterCallback(self, msg):
        """
        Gets the current state of the thrusters (values of all PWM channels)
        Topic: /auv/devices/thrusters
        msg: mavros_msgs.msg.OverrideRCIn
        """
        self.thrustTime = time.time()
        self.channels = list(msg.channels)
        channels = self.channels

        # If the sub is not armed, set the channels to neutral values
        if not self.armed:
            channels = [1500] * 18
        # If the time that the thrusters should have been thrusting for is exceeded by 1 sec, set to neutral values
        if time.time() - self.thrustTime > 1:
            channels = [1500] * 18

        msg.channels = channels
        
        # Publish thruster channels to RC override topic
        self.TOPIC_SET_RC_OVR.set_data(msg)
        self.topic_publisher(topic=self.TOPIC_SET_RC_OVR) 

        # update thruster time
        self.thrustTime = time.time()   

    def imuCallback(self, msg):
        """
        Get IMU from mavros and publish it to the AUV IMU topic
        Topic: /mavros/imu/data
        msg: sensor_msgs.msg.Imu   10Hz

        publish: /auv/devices/imu
        msg: sensor_msgs.msg.Imu   10Hz
        """
        data = msg
        self.AUV_IMU.set_data(data)
        self.topic_publisher(topic=self.AUV_IMU)

    def compassCallback(self, msg):
        # TODO need to add offset to compass heading
        # TODO test whether the compass offset is consistant
        """
        Get compass heading from mavros and publish it to the AUV compass topic
        Topic: /mavros/global_position/compass_hdg  3Hz
        msg: std_msgs.msg.Float64

        publish: /auv/devices/compass               3Hz
        msg: std_msgs.msg.Float64       
        """
        data = msg
        self.AUV_COMPASS.set_data(data)
        self.topic_publisher(topic=self.AUV_COMPASS)

    def baroCallback(self, msg):
        """
        Handles barometric data by unpacking, calculating depth from raw data, then publishes raw data
        Topic: /mavlink/from
        msg: mavros_msgs.msg.Mavlink        120Hz

        publish: /auv/devices/baro
        msg:  std_msgs.msg.Float64          2Hz   TODO test this again
        """
        try:
            # If the barometric data message has the right ID
            if msg.msgid == 143:
                # Unpack the data
                p = pack("QQ", *msg.payload64)
                time_boot_ms, press_abs, press_diff, temperature = unpack("Iffhxx", p) # Pressure is in mBar

                # Calculate the depth based on the pressure
                press_diff = round(press_diff, 2)
                press_abs = round(press_abs, 2)
                self.depth = (press_abs / (997.0474 * 9.80665 * 0.01)) - self.depth_calib

                # calibrate the depth with samples
                if not self.calibrate:
                    if(self.depth_sample_size >= len(self.depth_samples)):
                        self.depth_samples.append(self.depth)
                    else:
                        self.calibrate = True
                        self.depth_calib = mean(self.depth_samples)
                        print(f"[depth_calib] Finished. Surface is: {self.depth_calib}")

                # Publish the barometric data
                self.AUV_BARO.set_data(self.depth)
                self.topic_publisher(topic=self.AUV_BARO)

                time.sleep(1/20) # Attempt 20Hz
        # Handle exceptions
        except Exception as e:
            print("Baro Failed")
            print(e)
        
      
def onExit(signum, frame):
    """
    To handle exiting the function (gracefully)

    Args:
        signum: An integer representing the signal number
        frame: An object representing the current excution frame
    """
    try:
        # Disarm the sub, shut down ROS
        print("\nDisarming and exiting...")
        auv.arm(False)
        rospy.signal_shutdown("Rospy Exited")
        time.sleep(1)
        while not rospy.is_shutdown():
            pass
        print("\n\nCleanly Exited")
        exit(1)
    except:
        pass

# When Ctrl-C is pressed, call the OnExit() to exit the code
signal.signal(signal.SIGINT, onExit)

if __name__ == "__main__":
    """For running the script directly"""
    auv = AUV()
    
    
    