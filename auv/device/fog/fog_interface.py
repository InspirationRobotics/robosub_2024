#!/usr/bin/env python3

import time
import threading
import rospy
from std_msgs.msg import Float64
import serial
from auv.utils import deviceHelper

fog_port = deviceHelper.dataFromConfig("fog")

class FOG:
    def __init__(self, port='/dev/ttyUSB0'):
        rospy.init_node("fog_node", anonymous=True)

        self.ser = self._setupSerial(port)
        self.readData = False

        self.xDataNames = ["temp", "supply_voltage", "sld_curr", "diag_sig", "angle_deg"]
        self.data = {}
        self.parsed_data = {}

        self.samples = 200
        self.count = 0
        self.angle_sum = 0

        self.cal_time = 10
        self.cal_sum = 0
        self.cal_count = 0

        self.integration_factor = 0.0767
        self.integrated_sum = 0
        self.bias = 0

        # ROS publishers
        self.pub_angle = rospy.Publisher("/fog/angle_deg", Float64, queue_size=10)
        self.pub_temp = rospy.Publisher("/fog/temp", Float64, queue_size=10)
        self.pub_voltage = rospy.Publisher("/fog/supply_voltage", Float64, queue_size=10)
        self.pub_sld_curr = rospy.Publisher("/fog/sld_curr", Float64, queue_size=10)
        self.pub_diag_sig = rospy.Publisher("/fog/diag_sig", Float64, queue_size=10)

    def _setupSerial(self, p):
        return serial.Serial(
            port=p,
            baudrate=921600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )

    def start_read(self, func=None):
        self.reset_params()
        self.readData = True
        self.read_thread = threading.Thread(target=self._read_fog, args=(func,))
        self.read_thread.start()

    def stop_read(self):
        self.readData = False
        self.read_thread.join()
        rospy.loginfo("FOG read thread stopped.")

    def close(self):
        if self.readData:
            self.stop_read()
        self.ser.close()
        rospy.loginfo("FOG serial port closed.")

    def reset_params(self):
        self.data = {}
        self.parsed_data = {}

        self.count = 0
        self.angle_sum = 0
        self.integrated_sum = 0
        self.prev_time = time.time()

        self.cal_sum = 0
        self.cal_count = 0

    def reset_bias(self):
        self.bias = 0

    def calibrate(self):
        rospy.loginfo("Calibrating FOG... DO NOT TOUCH")
        self.reset_params()
        self.start_read(self._cal_fog_angle_data)
        time.sleep(self.cal_time)
        self.bias = self.cal_sum / (self.cal_count + 1e-5)
        self.stop_read()
        rospy.loginfo(f"Calibration complete. Bias: {self.bias:.4f}")

    def _handle_checksum(self, data):
        if len(data) != 8:
            return False
        try:
            checksum = sum(int(data[i], 16) for i in range(1, 6))
            high_cs = int(data[6], 16)
            low_cs = int(data[7], 16)
            return ((high_cs << 8) | low_cs) == checksum
        except:
            return False

    def _twos_complement(self, value):
        if value & (1 << 23):
            value -= (1 << 24)
        return value

    def _cal_fog_angle_data(self, curr_line, _):
        if not self._handle_checksum(curr_line):
            return
        angle_data = int(curr_line[2], 16) << 16 | int(curr_line[3], 16) << 8 | int(curr_line[1], 16)
        self.cal_sum += self._twos_complement(angle_data)
        self.cal_count += 1

    def _parse_fog_data(self, curr_line, prev_line):
        if not self._handle_checksum(curr_line) or not self._handle_checksum(prev_line):
            return

        packet_count = int(curr_line[4], 16)

        angle_data = int(curr_line[2], 16) << 16 | int(curr_line[3], 16) << 8 | int(curr_line[1], 16)
        self.angle_sum += (self._twos_complement(angle_data) - self.bias)
        self.count += 1

        if self.count >= self.samples:
            angle_mv = (self.angle_sum / self.samples) * (2.5 / (2**23))
            angle_deg_sec = angle_mv * self.integration_factor
            self.integrated_sum += angle_deg_sec * (time.time() - self.prev_time)
            self.parsed_data["angle_deg"] = self.integrated_sum
            self.prev_time = time.time()
            self.angle_sum = 0
            self.count = 0

        if packet_count % 2 != 0 and packet_count < 8:
            xData = int(prev_line[5], 16) << 8 | int(curr_line[5], 16)
            self.data[self.xDataNames[packet_count // 2]] = xData

        self._translate_data()
        self.publish_reading()

    def _translate_data(self):
        if "temp" in self.data:
            self.parsed_data["temp"] = self.data["temp"] * (250 / (2**15)) - 50
        if "supply_voltage" in self.data:
            self.parsed_data["supply_voltage"] = self.data["supply_voltage"] * (10 / (2**15))
        if "sld_curr" in self.data:
            self.parsed_data["sld_curr"] = self.data["sld_curr"] * (0.25 / (2**15))
        if "diag_sig" in self.data:
            self.parsed_data["diag_sig"] = self.data["diag_sig"] * (2.5 / (2**15))

    def _read_fog(self, func=None):
        line = []
        prev_line = []
        while self.readData:
            while self.ser.in_waiting:
                if not self.readData:
                    break
                byte = self.ser.read(1)
                if byte == b'\xdd':
                    if len(line) == 8 and (not prev_line or prev_line[4] != line[4]):
                        if func:
                            func(line, prev_line)
                        else:
                            self._parse_fog_data(line, prev_line)
                        prev_line = line
                    line = [byte.hex()]
                else:
                    line.append(byte.hex())

    def publish_reading(self):
        def pub_val(pub, key):
            if key in self.parsed_data:
                msg = Float64()
                msg.data = self.parsed_data[key]
                pub.publish(msg)

        pub_val(self.pub_angle, "angle_deg")
        pub_val(self.pub_temp, "temp")
        pub_val(self.pub_voltage, "supply_voltage")
        pub_val(self.pub_sld_curr, "sld_curr")
        pub_val(self.pub_diag_sig, "diag_sig")

if __name__ == "__main__":
    fog = FOG(fog_port)

    rospy.loginfo("Starting FOG calibration...")
    fog.calibrate()

    rospy.loginfo("FOG is now publishing data.")
    fog.start_read()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS shutdown signal received.")
    finally:
        fog.stop_read()
        fog.close()
