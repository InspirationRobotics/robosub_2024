#!/usr/bin/env python3

import rospy
import csv
import os
from sensor_msgs.msg import Imu


class ImuCollector:
    def __init__(self):
        rospy.init_node("imu_data_collector", anonymous=True)
        print("🛰️  IMU Data Collector Initialized")

        # Make sure the folder exists
        os.makedirs("imu_logs", exist_ok=True)

        # Open CSV file for writing
        filename = "imu_logs/imu_data.csv"
        self.csv_file = open(filename, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        # CSV Header
        self.csv_writer.writerow([
            "Time (ROS)",
            "Orientation_x", "Orientation_y", "Orientation_z", "Orientation_w",
            "AngularVel_x", "AngularVel_y", "AngularVel_z",
            "LinearAcc_x", "LinearAcc_y", "LinearAcc_z"
        ])

        # Subscribe to the IMU topic
        self.subscriber = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        rospy.on_shutdown(self.shutdown)

    def imu_callback(self, msg: Imu):
        # Extract IMU data
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration
        timestamp = msg.header.stamp.to_sec()

        # Write to CSV
        self.csv_writer.writerow([
            timestamp,
            orientation.x, orientation.y, orientation.z, orientation.w,
            angular_velocity.x, angular_velocity.y, angular_velocity.z,
            linear_acceleration.x, linear_acceleration.y, linear_acceleration.z
        ])

        rospy.loginfo_throttle(5, f"📥 IMU data logged at time {timestamp:.2f}")

    def shutdown(self):
        print("📦 Shutting down IMU collector... saving file.")
        self.csv_file.close()


if __name__ == "__main__":
    collector = ImuCollector()
    rospy.spin()
