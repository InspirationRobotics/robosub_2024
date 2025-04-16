#!/usr/bin/env python
import rospy
from struct import pack, unpack
import csv
import os
from mavros_msgs.msg import Mavlink
from dataCollect.filenameHelper import getFileName 

class BaroSubscriber:
    def __init__(self):
        rospy.init_node('baro_subscriber', anonymous=True)
        print("📡 BaroSubscriber initialized")
        self.subscriber = rospy.Subscriber('/mavlink/from', Mavlink, self.listener_callback)
        print("🔗 Subscribed to /mavlink/from")

        # Depth calibration (you can adjust this as needed)
        self.depth_calib = 0.0

        # Create a directory for CSV logs if it doesn't exist
        os.makedirs("baro_logs", exist_ok=True)

        # Open a CSV file to write the data
        self.csv_file = open(getFileName("barometer"), mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["Time_boot_ms", "Pressure_abs (mBar)", "Pressure_diff", "Temperature", "Depth (m)"])

    def listener_callback(self, msg):
        if msg.msgid == 143:
            try:
                # Unpack the data
                p = pack("QQ", *msg.payload64)
                time_boot_ms, press_abs, press_diff, temperature = unpack("Iffhxx", p)

                press_abs = round(press_abs, 2)
                press_diff = round(press_diff, 2)
                depth = (press_abs / (997.0474 * 9.80665 * 0.01)) - self.depth_calib
                depth = round(depth, 3)

                # Log to CSV
                self.csv_writer.writerow([time_boot_ms, press_abs, press_diff, temperature, depth])
                print(f"📝 Logged baro data — Depth: {depth:.2f}m")

            except Exception as e:
                rospy.logerr(f"❌ Failed to unpack baro message: {e}")

    def shutdown(self):
        print("🛑 Shutting down and saving CSV...")
        self.csv_file.close()


def main():
    baro_subscriber = BaroSubscriber()
    rospy.on_shutdown(baro_subscriber.shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()
