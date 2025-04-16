#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import csv
from threading import Thread
from queue import Queue
from dataCollect.filenameHelper import getFileName  # Your helper for naming files

class ImuSubscriber:
    def __init__(self):
        rospy.init_node('imu_collector', anonymous=True)

        # Prepare the CSV file and writer
        self.csv_path = getFileName("imu")
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["timestamp", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"])

        # Async queue for writing
        self.queue = Queue()
        self.shutdown_flag = False

        # Subscribe to the IMU topic
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.on_shutdown(self.shutdown)


        rospy.loginfo("🔁 IMU data collection started.")
        rospy.spin()

    def imu_callback(self, msg):
        # Get timestamp and sensor data
        timestamp = msg.header.stamp.to_sec()
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        if not self.shutdown_flag:
            self.writer.writerow([timestamp, ax, ay, az, gx, gy, gz])


    def shutdown(self):
        self.shutdown_flag = True
        self.csv_file.close()
        rospy.loginfo("🛑 Shutting down IMU logger...")
        rospy.loginfo(f"✅ CSV saved at: {self.csv_path}")


if __name__ == '__main__':
    imu = ImuSubscriber()
    rospy.on_shutdown(imu.shutdown)
