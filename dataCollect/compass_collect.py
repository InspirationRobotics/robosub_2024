#!/usr/bin/env python
import rospy
import std_msgs.msg
import csv
from sensor_msgs.msg import FluidPressure  # Although not used here, kept for consistency
import mavros_msgs.msg  # Importing only what is used
from dataCollect.filenameHelper import getFileName 

class CompassSubscriber():
    def __init__(self):
        rospy.init_node('compass_subscriber', anonymous=True)
        print("CompassSubscriber initialized")

        # Set up the subscriber to the /mavros/global_position/compass_hdg topic
        self.subscriber = rospy.Subscriber('/mavros/global_position/compass_hdg', std_msgs.msg.Float64, self.listener_callback)
        
        # Open the CSV file for writing and prepare the writer

        self.csv_filename = getFileName("compass")
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write the header row (optional)
        self.csv_writer.writerow(["timestamp", "compass_heading"])

        print("CompassSubscriber subscriber initialized")

    def listener_callback(self, msg):
        print("CompassSubscriber listener_callback called")
        
        # Extract the float value from the message
        compass_heading = msg.data  # msg.data contains the Float64 data
        
        # Get the timestamp for the message
        timestamp = rospy.get_time()

        # Write the data to the CSV file
        self.csv_writer.writerow([timestamp, compass_heading])
        print(f"Logged data to CSV: {timestamp}, {compass_heading}")

    def shutdown(self):
        # Close the CSV file gracefully
        print("Shutting down, closing CSV file...")
        self.csv_file.close()


def main():
    # Create an instance of the CompassSubscriber class
    compass_subscriber = CompassSubscriber()

    # Ensure proper shutdown on Ctrl+C
    rospy.on_shutdown(compass_subscriber.shutdown)
    
    rospy.spin()  # Keep the node running


if __name__ == '__main__':
    main()
