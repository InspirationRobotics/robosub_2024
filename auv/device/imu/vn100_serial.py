import time
import threading
from serial import Serial
import rospy
# import sensor_msgs.msg
import math
from auv.utils import deviceHelper


rospy.init_node("vectornav_api_node")

class VN100:
    def __init__(self,port:str = deviceHelper.dataFromConfig("vectornav")):
        """Makes a serial connection to the VN100 IMU utilizing deviceHelper"""
        self.__port = port
        self.__bps = 115200
        self.__ser = Serial(port=self.__port,baudrate=self.__bps, timeout=1)
    
    def get_orientation(self):
        """Parses roll, pitch, and yaw from the serial line"""
        # The format of a packet is first $YNYMR, then the yaw, pitch, roll;
        # then magnetometer, accelerometer, and gyroscope;
        # each in X, Y, then Z
        try:
            data_line = self.__ser.readline().decode()
            # I'll split it by commas to make accessing the data a bit easier
            
            data_list = data_line.split(',')
            self.yaw, self.pitch, self.roll = float(data_list[1]), float(data_list[2]), float(data_list[3])
        except IndexError:
            print("Bad data")


    
    
if __name__ == "__main__":
    sensor = VN100()

    # Make some variables to measure how often the IMU is changing its measurements
    imu_counter = 0
    imu_sample = []
    imu_angle_records = None
    # Collect data every second
    while True:
        try:
            sensor.get_orientation()
            print(f"Roll: {sensor.roll}\nPitch:{sensor.pitch}\nYaw:{sensor.yaw}\n")
            # Increment imu_counter
            imu_counter += 1
            # Check if any angles changed more than 5 degrees. First case populates
            # them for the first iteration
            if imu_angle_records is None:
                pass
            elif abs(sensor.roll - imu_angle_records[0]) > 5 or abs(sensor.pitch - imu_angle_records[1]) > 5 or abs(sensor.yaw - imu_angle_records[2]) > 5:
                imu_sample.append(imu_counter)
                imu_counter = 0
                print(imu_sample)
            # Populate angles
            imu_angle_records = [sensor.roll, sensor.pitch, sensor.yaw]
            time.sleep(1)
        except AttributeError:
            print("No data yet")
            time.sleep(1)
        except ValueError:
            print("Bad data")
        except Exception:
            print("Unanticipated event")
        except KeyboardInterrupt:
            print("Exiting")
            exit()


