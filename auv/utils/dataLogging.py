#!/usr/bin/env python
import os
import signal
import time
import csv
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
        self.csv_files = {}

    def generate_filename(self):
        now = datetime.now()
        time_str = now.strftime("%Y-%m-%d_%H-%M-%S")
        os.makedirs("/home/inspiration/bags", exist_ok=True)
        os.makedirs("/home/inspiration/csv_output", exist_ok=True)  # Create directory for CSVs
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
        # Set up subscribers for each topic
        self.setup_topic_and_csv("/auv/devices/compass", std_msgs.msg.Float64)
        self.setup_topic_and_csv("/auv/devices/imu", Imu)
        self.setup_topic_and_csv("/auv/devices/baro", std_msgs.msg.Float32MultiArray)
        self.setup_topic_and_csv("/auv/devices/thrusters", OverrideRCIn)
        self.setup_topic_and_csv("/auv/devices/setDepth", std_msgs.msg.Float64)
        self.setup_topic_and_csv("/auv/status/arm", std_msgs.msg.Bool)
        self.setup_topic_and_csv("/auv/status/mode", std_msgs.msg.String)

    def setup_topic_and_csv(self, topic, msg_type):
        # Create a CSV file for each topic
        csv_file = f"/home/inspiration/csv_output{topic.replace('/', '_')}.csv"
        csv_file = csv_file.replace(":", "_")
        self.csv_files[topic] = open(csv_file, 'w', newline='')
        self.csv_writers = {}
        self.csv_writers[topic] = csv.writer(self.csv_files[topic])

        rospy.Subscriber(topic, msg_type, self.bag_write_callback, callback_args=topic)

    def bag_write_callback(self, msg, topic):
        if self.bag:
            try:
                # Write to the bag
                self.bag.write(topic, msg)
                
                # Write to CSV
                if topic not in self.csv_writers:
                    return
                writer = self.csv_writers[topic]
                
                if writer.writerow == self.csv_writers[topic]:
                    # Write header for CSV file if it's the first message
                    headers = list(msg.__slots__)
                    writer.writerow(headers)
                
                # Write message data (fields) into CSV
                row = [getattr(msg, field) for field in msg.__slots__]
                writer.writerow(row)

            except Exception as e:
                rospy.logerr(f"Bag write error on {topic}: {e}")

    def shutdown(self):
        self.keep_running = False
        for topic, file in self.csv_files.items():
            file.close()


recorder = RosbagRecorder()

def handle_shutdown(sig, frame):
    print("\n🛑 Ctrl+C received, shutting down...")
    recorder.shutdown()

signal.signal(signal.SIGINT, handle_shutdown)

if __name__ == "__main__":
    recorder.start()
