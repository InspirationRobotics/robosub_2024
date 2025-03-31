"""
Attempting to get roll working. 

NOTE: We will have to create a new mode called "ACRO" in pix_standalone in order for this to even have a 
chance of working. 

NOTE: Roll rate is way too high, need to change to much lower to decrease risk of possible damage.
"""

#!/usr/bin/env python

# Importing necessary modules for platform information, signal handling, threading, time, statistical analysis, converting between python values and C structs (struct)
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

# For handling ROS topics
from ..utils.topicService import TopicService

# Different modes/states of travel (predefined modes used by the Pixhawk)
MODE_MANUAL = "MANUAL"
MODE_STABILIZE = "STABILIZE"
MODE_ALTHOLD = "ALT_HOLD"
MODE_LOITER = "LOITER"
MODE_AUTO = "AUTO"
MODE_GUIDED = "GUIDED"
MODE_ACRO = "ACRO"

class AUV:
    def __init__(self):
        rospy.init_node('auv_roll_node', anonymous=True)

        # Attributes relating to status
        self.do_publish_thrusters = True
        self.do_get_sensors = True
        self.armed = False
        self.guided = False
        self.mode = ""

        # Hold the depth
        self.depth = None
        self.do_hold_depth = False
        self.depth_pwm = 0
        self.depth_calib = 0
        self.depth_pid_params = [0.5, 0.1, 0.1]
        self.depth_pid_offset = 1500
        self.depth_pid = PID(*self.depth_pid_params, setpoint=0.65)
        self.depth_pid.output_limits = (-self.depth_pid_params[0], self.depth_pid_params[0])

        # Initialize ROS topics and services
        self.TOPIC_STATE = rospy.Subscriber("/mavros/state", mavros_msgs.msg.State, self.update_parameters_from_topic)
        self.SERVICE_ARM = rospy.ServiceProxy("/mavros/cmd/arming", mavros_msgs.srv.CommandBool)
        self.SERVICE_SET_MODE = rospy.ServiceProxy("/mavros/set_mode", mavros_msgs.srv.SetMode)
        self.SERVICE_SET_PARAM = rospy.ServiceProxy("/mavros/param/set", mavros_msgs.srv.ParamSet)
        self.SERVICE_GET_PARAM = rospy.ServiceProxy("/mavros/param/get", mavros_msgs.srv.ParamGet)

        self.TOPIC_SET_VELOCITY = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", geometry_msgs.msg.Twist, queue_size=10)
        self.TOPIC_SET_RC_OVR = rospy.Publisher("/mavros/rc/override", mavros_msgs.msg.OverrideRCIn, queue_size=10)

        self.TOPIC_GET_IMU_DATA = rospy.Subscriber("/mavros/imu/data", sensor_msgs.msg.Imu, self.get_imu_data)
        self.TOPIC_GET_CMP_HDG = rospy.Subscriber("/mavros/global_position/compass_hdg", std_msgs.msg.Float64, self.get_compass_heading)
        self.TOPIC_GET_RC = rospy.Subscriber("/mavros/rc/in", mavros_msgs.msg.RCIn, self.get_rc_data)
        self.TOPIC_GET_MAVBARO = rospy.Subscriber("/mavlink/from", mavros_msgs.msg.Mavlink, self.get_baro)
        self.TOPIC_GET_BATTERY = rospy.Subscriber("/mavros/battery", sensor_msgs.msg.BatteryState, self.get_battery_data)

        self.AUV_GET_DEPTH = rospy.Subscriber("/auv/devices/setDepth", std_msgs.msg.Float64, self.set_depth)
        self.AUV_GET_REL_DEPTH = rospy.Subscriber("/auv/devices/setRelativeDepth", std_msgs.msg.Float64, self.set_rel_depth)
        self.AUV_GET_ARM = rospy.Subscriber("/auv/status/arm", std_msgs.msg.Bool, self.set_arm)
        self.AUV_GET_MODE = rospy.Subscriber("/auv/status/mode", std_msgs.msg.String, self.set_mode)

        self.depth_calib = 0
        self.depth = None

    def arm(self, status):
        try:
            res = self.SERVICE_ARM(value=status)
            self.armed = res.success
            return res.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Arming failed: {e}")
            return False

    def change_mode(self, mode):
        try:
            res = self.SERVICE_SET_MODE(custom_mode=mode)
            self.mode = mode
            return res.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr(f"Set mode failed: {e}")
            return None

    def calibrate_depth(self, sample_time=3):
        samples = []

        while self.depth is None:
            rospy.sleep(0.1)

        prev_depth = self.depth
        start_time = time.time()

        while time.time() - start_time < sample_time:
            if self.depth == prev_depth:
                continue

            samples.append(self.depth)
            prev_depth = self.depth

        self.depth_calib = mean(samples)

    def depth_hold(self, depth):
        if depth < -9 or depth > 100:
            return
        self.depth_pwm = int(self.depth_pid(depth) * -1 + self.depth_pid_offset)
        rospy.loginfo(f"Depth: {depth:.4f}, DepthMotorPower: {self.depth_pwm}, Target: {self.depth_pid.setpoint}")

    def get_baro(self, baro):
        if baro.msgid == 143:
            p = pack("QQ", *baro.payload64)
            time_boot_ms, press_abs, press_diff, temperature = unpack("Iffhxx", p)
            press_diff = round(press_diff, 2)
            press_abs = round(press_abs, 2)
            self.depth = (press_abs / (997.0474 * 9.80665 * 0.01)) - self.depth_calib
            if self.do_hold_depth and self.armed:
                self.depth_hold(self.depth)

    def set_depth(self, depth):
        if depth.data < 0:
            return
        self.depth_pid.setpoint = depth.data

    def set_rel_depth(self, relative_depth):
        new_depth = self.depth_pid.setpoint + relative_depth.data
        if new_depth < 0:
            return
        self.depth_pid.setpoint = new_depth

    def set_arm(self, msg):
        self.arm(msg.data)

    def set_mode(self, msg):
        self.change_mode(msg.data)

    def get_imu_data(self, data):
        self.imu = data

    def get_compass_heading(self, data):
        self.hdg = data

    def get_rc_data(self, data):
        self.rc = data

    def get_battery_data(self, data):
        self.battery = data

    def perform_roll(self, roll_angle=360, roll_rate_deg_per_sec=60):
        self.calibrate_depth()
        target_depth = self.depth
        self.depth_pid.setpoint = target_depth
        self.do_hold_depth = True

        self.change_mode(MODE_ACRO)
        rospy.sleep(1)

        roll_duration = roll_angle / roll_rate_deg_per_sec
        roll_rate_rad_per_sec = np.deg2rad(roll_rate_deg_per_sec)

        twist = geometry_msgs.msg.Twist()
        twist.angular.x = roll_rate_rad_per_sec

        start_time = time.time()
        while time.time() - start_time < roll_duration:
            self.TOPIC_SET_VELOCITY.publish(twist)
            self.depth_hold(self.depth)
            rospy.sleep(0.1)

        self.change_mode(MODE_STABILIZE)
        self.do_hold_depth = False

def main():
    auv = AUV()

    while not rospy.is_shutdown():
        if auv.perform_roll():
            rospy.loginfo("Roll completed successfully.")
        rospy.sleep(5)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
