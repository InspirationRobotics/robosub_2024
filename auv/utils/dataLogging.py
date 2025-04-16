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
        self.bag = None
        self.keep_running = True

    def generate_filename(self):
        now = datetime.now()
        time_str = now.strftime("%Y-%m-%d_%H-%M-%S")
        os.makedirs("/home/inspiration/bags", exist_ok=True)
        return f"/home/inspiration/bags/{time_str}.bag"

    def start(self):
        filename = self.generate_filename()
        print(f"📦 Creating bag file: {filename}")
        with rosbag.Bag(filename, 'w') as self.bag:
            self.setup_subscribers()
            print("🟢 Recording started. Press Ctrl+C to stop.")
            while not rospy.is_shutdown() and self.keep_running:
                time.sleep(0.1)
        print(f"✅ Bag saved and indexed at: {filename}")

    def setup_subscribers(self):
        rospy.Subscriber("/auv/devices/compass", std_msgs.msg.Float64, self.bag_write_callback, "/auv/devices/compass")
        rospy.Subscriber("/auv/devices/imu", Imu, self.bag_write_callback, "/auv/devices/imu")
        rospy.Subscriber("/auv/devices/baro", std_msgs.msg.Float32MultiArray, self.bag_write_callback, "/auv/devices/baro")
        rospy.Subscriber("/auv/devices/thrusters", OverrideRCIn, self.bag_write_callback, "/auv/devices/thrusters")
        rospy.Subscriber("/auv/devices/setDepth", std_msgs.msg.Float64, self.bag_write_callback, "/auv/devices/setDepth")
        rospy.Subscriber("/auv/status/arm", std_msgs.msg.Bool, self.bag_write_callback, "/auv/status/arm")
        rospy.Subscriber("/auv/status/mode", std_msgs.msg.String, self.bag_write_callback, "/auv/status/mode")

    def bag_write_callback(self, msg, topic):
        if self.bag:
            try:
                self.bag.write(topic, msg)
            except Exception as e:
                rospy.logerr(f"Bag write error on {topic}: {e}")

    def shutdown(self):
        self.keep_running = False


recorder = RosbagRecorder()

def handle_shutdown(sig, frame):
    print("\n🛑 Ctrl+C received, shutting down...")
    recorder.shutdown()

signal.signal(signal.SIGINT, handle_shutdown)

if __name__ == "__main__":
    recorder.start()
