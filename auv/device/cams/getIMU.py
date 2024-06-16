"""
Code to print out the output from the gyroscope and acclerometer present inside an OAK-D Wide/Pro Wide camera.
X-axis is rotation around a vertical axis, which is what is important.
"""
# TODO: Code works, but prints very slowly -- need to figure out how to make that more efficient, then 
# convert to absolute degree change.

#!/usr/bin/env python3

import cv2
import depthai as dai
import time
import math

def timeDeltaToMilliS(delta) -> float:
    return delta.total_seconds()*1000



class oakIMU:
    def __init__(self, accelerometer):
        self.accelerometer = accelerometer # True means get accelerometer data, false means don't.

        self.heading = 0 # Relative heading from start in degrees

         # Create pipeline
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        imu = self.pipeline.create(dai.node.IMU)
        xlinkOut = self.pipeline.create(dai.node.XLinkOut)

        xlinkOut.setStreamName("imu")

        # enable ACCELEROMETER_RAW at 500 hz rate
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 1)
        # enable GYROSCOPE_RAW at 400 hz rate
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 1)
        # it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
        # above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
        imu.setBatchReportThreshold(1)
        # maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
        # if lower or equal to batchReportThreshold then the sending is always blocking on device
        # useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
        imu.setMaxBatchReports(10)

        # Link plugins IMU -> XLINK
        imu.out.link(xlinkOut.input)

        # Make angular position in degrees

        self.angles = {
        "x": 0,
        "y": 0,
        "z": 0
        }

        self.curr_time = None
        self.prev_time = None

    def get_raw_data(self, get_accelerometer):

        print("I ran a new connection")
        # Pipeline is defined, now we can connect to the device
        with dai.Device(self.pipeline) as device:

            # Output queue for imu bulk packets
            imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
            print("I made a queue")
            baseTs = None

            while True:
                imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

                imuPackets = imuData.packets
                for imuPacket in imuPackets:
                    self.acceleroValues = imuPacket.acceleroMeter
                    self.gyroValues = imuPacket.gyroscope

                    acceleroTs = self.acceleroValues.getTimestampDevice()
                    gyroTs = self.gyroValues.getTimestampDevice()
                    if baseTs is None:
                        baseTs = acceleroTs if acceleroTs < gyroTs else gyroTs
                    self.acceleroTs = timeDeltaToMilliS(acceleroTs - baseTs)
                    self.gyroTs = timeDeltaToMilliS(gyroTs - baseTs)

                    self.imuF = "{:.06f}"
                    self.tsF  = "{:.03f}"

                # Units for data:
                # Timestamps in ms
                # Acceleration in m/s^2
                # Angular velocity (gyro) in rad/s
    
                accelerometer_dict = {}
                gyro_dict = {}

                gyro_dict = {
                        "Time" : self.tsF.format(self.gyroTs),
                        "Gyro_x" : self.imuF.format(self.gyroValues.x), # Most of interest to us
                        "Gyro_y" : self.imuF.format(self.gyroValues.y),
                        "Gyro_z" : self.imuF.format(self.gyroValues.z)
                }

                # Modify anglular position through integrating
                # angular velocity - time in milliseconds!

                self.curr_time = float(gyro_dict["Time"])

                if self.prev_time is None:
                    self.prev_time = self.curr_time
                    print("[WARN] Need one more packet",
                          "position is currently 0")

                dt = (self.curr_time - self.prev_time) / 1000

                self.prev_time = self.curr_time

                self.angles = {
                    "x": (self.angles["x"] + math.degrees(float(gyro_dict["Gyro_x"])) * dt) % 360,
                    "y": (self.angles["y"] + math.degrees(float(gyro_dict["Gyro_y"])) * dt) % 360,
                    "z": (self.angles["z"] + math.degrees(float(gyro_dict["Gyro_z"])) * dt) % 360
                }
                # Return data

                if get_accelerometer == True:
                    accelerometer_dict = {
                        "Accel_time" : self.tsF.format(self.acceleroTs), 
                        "Accel_x" : self.imuF.format(self.acceleroValues.x),
                        "Accel_y" : self.imuF.format(self.acceleroValues.y),
                        "Accel_z" : self.imuF.format(self.acceleroValues.z)
                    }
                    yield accelerometer_dict, gyro_dict
                    
                elif get_accelerometer == False:
                    yield self.angles
                    
    def radians_to_degrees(self, rad_measure):
        return math.degrees(rad_measure)
        
    def absolute_rotation(self):
        accelerometer_data = None
        gyro_data = None

        # Yield statements in get_raw_data 
        # allow for only one variable to take the data

        if self.accelerometer == True:
            accelerometer_data = self.get_raw_data(self.accelerometer)
        elif self.accelerometer == False:
            gyro_data = self.get_raw_data(self.accelerometer)

        if accelerometer_data is not None:
            # This part is not functional right now - NOT TESTED
            for line in accelerometer_data:
                yield line
        else:
            # Gyroscope_data is now a generator, must be accessed w/ for loop
            for line in gyro_data:
                yield line

if __name__ == "__main__":
    oakIMU = oakIMU(False)   
    while True:
        if oakIMU.accelerometer == True:
            accelerometer_data, gyroscope_data = oakIMU.absolute_rotation()
            print(f"Accelerometer data : {accelerometer_data} \n Gyroscope data : {gyroscope_data}")
        else:
            angle_data = oakIMU.absolute_rotation()
            print(f"Angle data : ", end='')

            # Be warned that the gyro angles have 3-4 second latency

            for line in angle_data:
                print(line)
                time.sleep(0.1)
        if cv2.waitKey(1) == ord('q'):
            break
