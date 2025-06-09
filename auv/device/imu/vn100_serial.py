import time
import threading
from serial import Serial
from auv.utils import deviceHelper

class VN100:
    def __init__(self,port:str = deviceHelper.dataFromConfig("vectornav")): # deviceHelper.dataFromConfig("vectornav")
        """Makes a serial connection to the VN100 IMU utilizing deviceHelper and starts reading"""
        self.__port = port
        self.__bps = 115200
        self.__ser = Serial(port=self.__port,baudrate=self.__bps, timeout=1)

        # Initialize sensor values
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.accX = 0.0
        self.accY = 0.0
        self.accZ = 0.0
        self.gyroX = 0.0
        self.gyroY = 0.0
        self.gyroZ = 0.0

        # start thread
        self.read_thread = threading.Thread(target=self.read, daemon=True)
        self.read_thread.start()

        time.sleep(2) # sleep for 2s and wait for one iteration in thread
    
    def read(self):
        """Parses roll, pitch, and yaw from the serial line"""
        # The format of a packet is first $YNYMR, then the yaw, pitch, roll;
        # then magnetometer, accelerometer, and gyroscope;
        # each in X, Y, then Z

        # Do a while statement to make the loop run forever
        while True:
            time.sleep(1/100)
            try:
                # print("thread run")
                # Read data
                data_line = self.__ser.readline().decode()

                # I'll split it by commas to make accessing the data a bit easier
                data_list = data_line.split(',')

                # debug
                # print(data_list)
        
                # Populate yaw, pitch, roll
                self.yaw, self.pitch, self.roll = (float(data_list[1]) + 90) % 360, float(data_list[3]), float(data_list[2])
                self.accX, self.accY, self.accZ = float(data_list[4]) , float(data_list[5]), float(data_list[6])
                self.gyroX, self.gyroY, self.gyroZ = float(data_list[7]) , float(data_list[8]), float(data_list[9])

            except IndexError:
                print("Bad data")
            except Exception:
                pass


    
    
if __name__ == "__main__":
    import time
    import csv
    from datetime import datetime
    sensor = VN100()

    # Create a timestamped filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"data_{timestamp}.csv"

    data = []
    print("Recording... Press Ctrl+C to stop.")

    try:
        while True:
            # time.sleep(1 / 50)  # 50 Hz
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
            #print(f"Acc: ({sensor.accX:.2f}, {sensor.accY:.2f}, {sensor.accZ:.2f})")
            #print(f"Gyro: ({sensor.gyroX:.2f}, {sensor.gyroY:.2f}, {sensor.gyroZ:.2f})")

    except KeyboardInterrupt:
        print("Exiting and saving data to CSV...")
        fieldnames = ["timestamp", "Roll", "Pitch", "Yaw", "AccX","AccY","AccZ"]

        with open(filename, mode="w", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(data)

        print(f"Data saved to {filename}")

    except AttributeError:
        print("No data yet")
    except ValueError:
        print("Bad data")
    except Exception as e:
        print(f"Generic exception caught: {e}")


