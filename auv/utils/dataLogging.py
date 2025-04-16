#!/usr/bin/env python
import os
import signal
import time
from datetime import datetime

import rosbag
import rospy
import std_msgs.msg
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import FluidPressure, Imu


class RosbagRecorder:
    def __init__(self):
        rospy.init_node("rosbag_creator", anonymous=True)
        self.name = self.generate_filename()
        os.makedirs("/home/inspiration/bags", exist_ok=True)
        self.bag = rosbag.Bag(self.name, "w")
        print(f"✅ Successfully created bag file: {self.name}")

        # Subscribers
        self.subs = []
        self.subs.append(rospy.Subscriber("/auv/devices/compass", std_msgs.msg.Float64, self.bag_write_callback, "/auv/devices/compass"))
        self.subs.append(rospy.Subscriber("/auv/devices/imu", Imu, self.bag_write_callback, "/auv/devices/imu"))
        self.subs.append(rospy.Subscriber("/auv/devices/baro", std_msgs.msg.Float32MultiArray, self.bag_write_callback, "/auv/devices/baro"))
        self.subs.append(rospy.Subscriber("/auv/devices/thrusters", OverrideRCIn, self.bag_write_callback, "/auv/devices/thrusters"))
        self.subs.append(rospy.Subscriber("/auv/devices/setDepth", std_msgs.msg.Float64, self.bag_write_callback, "/auv/devices/setDepth"))
        self.subs.append(rospy.Subscriber("/auv/status/arm", std_msgs.msg.Bool, self.bag_write_callback, "/auv/status/arm"))
        self.subs.append(rospy.Subscriber("/auv/status/mode", std_msgs.msg.String, self.bag_write_callback, "/auv/status/mode"))

    def generate_filename(self):
        now = datetime.now()
        time_str = now.strftime("%Y-%m-%d_%H-%M-%S")
        return f"/home/inspiration/bags/{time_str}.bag"

    def bag_write_callback(self, msg, topic):
        try:
            self.bag.write(topic, msg)
        except Exception as e:
            rospy.logerr(f"Failed to write to bag: {e}")

    def close(self):
        print("\n🛑 Shutting down... closing bag file.")
        self.bag.close()
        print(f"📁 Bag saved to {self.name}")


# Global recorder instance
recorder = None

def handle_shutdown(sig, frame):
    global recorder
    if recorder:
        recorder.close()
    rospy.signal_shutdown("Shutdown requested")
    exit(0)

signal.signal(signal.SIGINT, handle_shutdown)

if __name__ == "__main__":
    recorder = RosbagRecorder()

    try:
        # Main loop keeps script alive and responsive to shutdown
        while not rospy.is_shutdown():
            time.sleep(0.1)
    except rospy.ROSInterruptException:
        pass
