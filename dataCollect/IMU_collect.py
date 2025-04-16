import rospy
from sensor_msgs.msg import Imu
import csv
import os
from threading import Thread
from queue import Queue
from datetime import datetime
from dataCollect.filenameHelper import getFileName 

class ImuSubscriber:
    def __init__(self):
        rospy.init_node('imu_collector', anonymous=True)

        self.csv_filename = getFileName("compass_data")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["timestamp", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"])

        # Async queue for writing
        self.queue = Queue()
        self.shutdown_flag = False

        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.on_shutdown(self.shutdown)

        self.writer_thread = Thread(target=self.csv_writer)
        self.writer_thread.start()

        rospy.loginfo("🔁 IMU data collection started.")
        rospy.spin()

    def imu_callback(self, msg):
        # Timestamp and IMU readings
        timestamp = msg.header.stamp.to_sec()
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        self.queue.put([timestamp, ax, ay, az, gx, gy, gz])

    def csv_writer(self):
        while not self.shutdown_flag or not self.queue.empty():
            try:
                row = self.queue.get(timeout=0.1)
                self.writer.writerow(row)
            except:
                continue

    def shutdown(self):
        rospy.loginfo("🛑 Shutting down IMU logger...")
        self.shutdown_flag = True
        self.writer_thread.join()
        self.csv_file.close()
        rospy.loginfo(f"✅ CSV saved at: {self.csv_path}")


if __name__ == '__main__':
    imu = ImuSubscriber()
    rospy.on_shutdown(imu.shutdown)


