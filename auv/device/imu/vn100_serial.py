import rospy
import time
import threading

from serial import Serial
from transforms3d.euler import euler2quat

from auv.utils import deviceHelper
from sensor_msgs.msg import Imu

rospy.init_node("vectornav_serial_api", anonymous=True)

class VN100:
    def __init__(self, port: str = deviceHelper.dataFromConfig("vectornav")):
        """Makes a serial connection to the VN100 IMU utilizing deviceHelper and starts reading"""
        self.__port = port
        self.__bps = 115200
        self.__ser = Serial(port=self.__port, baudrate=self.__bps, timeout=1)

        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.accX = 0.0
        self.accY = 0.0
        self.accZ = 0.0
        self.gyroX = 0.0
        self.gyroY = 0.0
        self.gyroZ = 0.0
        self.vectornav_pub = rospy.Publisher('/auv/devices/vectornav', Imu, queue_size=10)

        self.running = True  # Added for Ctrl+C protection

        self.read_thread = threading.Thread(target=self.read, daemon=True)
        self.read_thread.start()

        self.publish_thread = threading.Thread(target=self.publish_data, daemon=True)
        self.publish_thread.start()

        time.sleep(2)

    def read(self):
        """Parses roll, pitch, and yaw from the serial line"""
        while self.running:
            time.sleep(1 / 100)
            try:
                data_line = self.__ser.readline().decode()
                data_list = data_line.split(',')

                self.yaw, self.pitch, self.roll = (float(data_list[1]) + 90) % 360, float(data_list[3]), float(data_list[2])
                self.get_orientation()

                self.accX, self.accY, self.accZ = float(data_list[4]), float(data_list[5]), float(data_list[6])
                self.gyroX, self.gyroY, self.gyroZ = float(data_list[7]), float(data_list[8]), float(data_list[9])

            except IndexError:
                print("Bad data")
            except Exception:
                pass

    def get_orientation(self):
        """Converts Euler angles to quaternion form"""
        self.quat_orient = euler2quat(self.pitch, self.roll, self.yaw)

    def publish_data(self):
        """Published the IMU data to /auv/devices/vectornav"""
        while self.running and not rospy.is_shutdown():
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "vectornav"

            imu_msg.orientation.w = self.quat_orient[0]
            imu_msg.orientation.x = self.quat_orient[1]
            imu_msg.orientation.y = self.quat_orient[2]
            imu_msg.orientation.z = self.quat_orient[3]

            imu_msg.angular_velocity.x = self.gyroX
            imu_msg.angular_velocity.y = self.gyroY
            imu_msg.angular_velocity.z = self.gyroZ

            imu_msg.linear_acceleration.x = self.accX
            imu_msg.linear_acceleration.y = self.accY
            imu_msg.linear_acceleration.z = self.accZ

            self.vectornav_pub.publish(imu_msg)

    def shutdown(self):
        """Clean shutdown of threads and serial connection"""
        self.running = False
        if self.__ser.is_open:
            self.__ser.close()


if __name__ == "__main__":
    import csv
    from datetime import datetime

    sensor = VN100()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"data_{timestamp}.csv"

    data = []
    print("Recording... Press Ctrl+C to stop.")

    try:
        while True:
            data.append({
                "timestamp": datetime.now().isoformat(),
                "Roll": sensor.roll,
                "Pitch": sensor.pitch,
                "Yaw": sensor.yaw,
                "AccX": sensor.accX,
                "AccY": sensor.accY,
                "AccZ": sensor.accZ
            })
            print(f"YPR: ({sensor.yaw:.2f}, {sensor.pitch:.2f}, {sensor.roll:.2f})")

    except KeyboardInterrupt:
        sensor.shutdown()
        rospy.loginfo("shutting down vn100 node")
        # print("Exiting and saving data to CSV...")
        # fieldnames = ["timestamp", "Roll", "Pitch", "Yaw", "AccX", "AccY", "AccZ"]
        # with open(filename, mode="w", newline="") as csvfile:
        #     writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        #     writer.writeheader()
        #     writer.writerows(data)
        # print(f"Data saved to {filename}")

    except AttributeError:
        print("No data yet")
    except ValueError:
        print("Bad data")
    except Exception as e:
        print(f"Generic exception caught: {e}")
