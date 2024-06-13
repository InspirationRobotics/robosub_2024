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

class oakIMU:
    def __init__(self, accelerometer):
        self.accelerometer = accelerometer # True means get accelerometer data, false means don't.

        self.heading = 0 # Relative heading from start in degrees

    def get_raw_data(self, get_accelerometer):
        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        imu = pipeline.create(dai.node.IMU)
        xlinkOut = pipeline.create(dai.node.XLinkOut)

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

        # Pipeline is defined, now we can connect to the device
        with dai.Device(pipeline) as device:

            def timeDeltaToMilliS(delta) -> float:
                return delta.total_seconds()*1000

            # Output queue for imu bulk packets
            imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
            baseTs = None
            while True:
                imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

                imuPackets = imuData.packets
                for imuPacket in imuPackets:
                    acceleroValues = imuPacket.acceleroMeter
                    gyroValues = imuPacket.gyroscope

                    acceleroTs = acceleroValues.getTimestampDevice()
                    gyroTs = gyroValues.getTimestampDevice()
                    if baseTs is None:
                        baseTs = acceleroTs if acceleroTs < gyroTs else gyroTs
                    acceleroTs = timeDeltaToMilliS(acceleroTs - baseTs)
                    gyroTs = timeDeltaToMilliS(gyroTs - baseTs)

                    imuF = "{:.06f}"
                    tsF  = "{:.03f}"

                    accelerometer_dict = {}
                    gyroscope_dict = []

                    if get_accelerometer == True:
                        accelerometer_dict = {
                            "Accelerometer timestamp in ms" : tsF.format(acceleroTs), 
                            "Accelerometer [m/s^2] x-axis" : imuF.format(acceleroValues.x),
                            "Accelerometer [m/s^2] y-axis" : imuF.format(acceleroValues.y),
                            "Accelerometer [m/s^2] z-axis" : imuF.format(acceleroValues.z)
                        }
                        
                        gyroscope_dict = {
                            "Gyroscope timestamp in ms" : tsF.format(gyroTs),
                            "Gyroscope [rad/s] x-axis" : imuF.format(gyroValues.x), # Most of interest to us
                            "Gyroscope [rad/s] y-axis" : imuF.format(gyroValues.y),
                            "Gyroscope [rad/s] z-axis" : imuF.format(gyroValues.z)
                        }

                        return accelerometer_dict, gyroscope_dict
                    
                    elif get_accelerometer == False:
                        gyroscope_dict = {
                            "Gyroscope timestamp in ms" : tsF.format(gyroTs),
                            "Gyroscope [rad/s] x-axis" : imuF.format(gyroValues.x), # Most of interest to us
                            "Gyroscope [rad/s] y-axis" : imuF.format(gyroValues.y),
                            "Gyroscope [rad/s] z-axis" : imuF.format(gyroValues.z)
                        }
                        return gyroscope_dict
                    
    def radians_to_degrees(self, rad_measure):
        return math.degrees(rad_measure)
    def absolute_rotation(self):
        accelerometer_data = None
        gyroscope_data = None

        if self.accelerometer == True:
            accelerometer_data, gyroscope_data = self.get_raw_data(self.accelerometer)
        elif self.accelerometer == False:
            gyroscope_data = self.get_raw_data(self.accelerometer)

        if accelerometer_data is not None:
            return accelerometer_data, gyroscope_data
        else:
            return gyroscope_data

if __name__ == "__main__":
    oakIMU = oakIMU(False)   
    while True:
        if oakIMU.accelerometer == True:
            accelerometer_data, gyroscope_data = oakIMU.absolute_rotation()
            print(f"Accelerometer data : {accelerometer_data} \n Gyroscope data : {gyroscope_data}")
        else:
            gyroscope_data = oakIMU.absolute_rotation()
            print(f"Gyroscope data : {gyroscope_data}")
        if cv2.waitKey(1) == ord('q'):
            break