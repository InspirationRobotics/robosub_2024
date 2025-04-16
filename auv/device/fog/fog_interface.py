#!/usr/bin/env python3

import time
import rospy
import serial
from std_msgs.msg import Float64
from auv.utils import deviceHelper

fog_port = deviceHelper.dataFromConfig("fog")

class FOGNode:
    def __init__(self, port='/dev/ttyUSB0'):
        rospy.init_node("fog_node", anonymous=True)
        self.ser = self._setupSerial(port)

        self.data = {}
        self.parsed_data = {}

        self.xDataNames = ["temp", "supply_voltage", "sld_curr", "diag_sig", "angle_deg"]

        self.samples = 200
        self.count = 0
        self.angle_sum = 0

        self.cal_time = 10
        self.cal_sum = 0
        self.cal_count = 0

        self.integration_factor = 0.0767
        self.integrated_sum = 0
        self.bias = 0
        self.prev_time = time.time()

        # ROS publishers
        self.pub_angle = rospy.Publisher("/fog/angle_deg", Float64, queue_size=10)
        self.pub_temp = rospy.Publisher("/fog/temp", Float64, queue_size=10)
        self.pub_voltage = rospy.Publisher("/fog/supply_voltage", Float64, queue_size=10)
        self.pub_sld_curr = rospy.Publisher("/fog/sld_curr", Float64, queue_size=10)
        self.pub_diag_sig = rospy.Publisher("/fog/diag_sig", Float64, queue_size=10)

        # Calibration first
        self.calibrate()

        # Start Timer-based read loop
        self.line_buffer = []
        self.prev_line = []
        rospy.Timer(rospy.Duration(0.001), self.read_callback)

    def _setupSerial(self, p):
        return serial.Serial(
            port=p,
            baudrate=921600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )

    def calibrate(self):
        rospy.loginfo("Calibrating FOG... DO NOT TOUCH")
        self.cal_sum = 0
        self.cal_count = 0
        start_time = time.time()
        self.line_buffer = []
        self.prev_line = []

        while time.time() - start_time < self.cal_time and not rospy.is_shutdown():
            self._read_serial_line(self._cal_fog_angle_data)

        self.bias = self.cal_sum / (self.cal_count + 1e-5)
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
            now = time.time()
            dt = now - self.prev_time
            self.integrated_sum += angle_deg_sec * dt
            self.parsed_data["angle_deg"] = self.integrated_sum
            self.prev_time = now
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

    def _read_serial_line(self, callback):
        while self.ser.in_waiting:
            byte = self.ser.read(1)
            if byte == b'\xdd':
                if len(self.line_buffer) == 8 and (not self.prev_line or self.prev_line[4] != self.line_buffer[4]):
                    callback(self.line_buffer, self.prev_line)
                    self.prev_line = self.line_buffer
                self.line_buffer = [byte.hex()]
            else:
                self.line_buffer.append(byte.hex())

    def read_callback(self, event):
        self._read_serial_line(self._parse_fog_data)

    def publish_reading(self):
        def pub(pub, key):
            if key in self.parsed_data:
                pub.publish(Float64(data=self.parsed_data[key]))

        pub(self.pub_angle, "angle_deg")
        pub(self.pub_temp, "temp")
        pub(self.pub_voltage, "supply_voltage")
        pub(self.pub_sld_curr, "sld_curr")
        pub(self.pub_diag_sig, "diag_sig")

    def shutdown(self):
        self.ser.close()

if __name__ == "__main__":

    fog_node = FOGNode(fog_port)
    rospy.spin()
    rospy.on_shutdown(fog_node.shutdown)

