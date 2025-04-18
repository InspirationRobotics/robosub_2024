import time
import threading
import rospy
from std_msgs.msg import Float64
import serial
from auv.utils import deviceHelper


class FOG:
    """Fiber Optic Gyroscope reader with ROS integration"""

    def __init__(self, port='/dev/ttyUSB0'):
        rospy.init_node("FOG", anonymous=True)

        self.ser = self._setup_serial(port)
        self.readData = False

        self.xDataNames = ["temp", "supply_voltage", "sld_curr", "diag_sig", "angle_deg"]
        self.data = {}
        self.parsed_data = {}

        self.samples = 200
        self.count = 0
        self.angle_sum = 0

        self.cal_time = 10  # seconds
        self.cal_sum = 0
        self.cal_count = 0

        self.integration_factor = 0.0767  # deg/sec/mV
        self.integrated_sum = 0
        self.bias = 0

        self.pub_fog_ang = rospy.Publisher("auv/devices/fog/ang", Float64, queue_size=10)
        self.pub_fog_angvel = rospy.Publisher("auv/devices/fog/angvel", Float64, queue_size=10)

    def _setup_serial(self, port):
        return serial.Serial(
            port=port,
            baudrate=921600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )

    def start_read(self, callback=None):
        self.reset_params()
        self.readData = True
        self.read_thread = threading.Thread(target=self._read_fog, args=(callback,))
        self.read_thread.start()

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
        print("Calibrating FOG... DO NOT TOUCH")
        self.reset_params()
        self.start_read(self._cal_fog_angle_data)
        time.sleep(self.cal_time)
        self.bias = self.cal_sum / (self.cal_count + 1e-5)
        self.stop_read()
        print(f"Calibration complete. Bias: {self.bias:.4f}")

    def _handle_checksum(self, data):
        if len(data) != 8:
            return False
        checksum = sum(int(data[i], 16) for i in range(1, 6))
        high_cs = int(data[6], 16)
        low_cs = int(data[7], 16)
        return ((high_cs << 8) | low_cs) == checksum

    def _twos_complement(self, value):
        if value & (1 << 23):
            value -= (1 << 24)
        return value

    def _cal_fog_angle_data(self, curr_line, _):
        if not self._handle_checksum(curr_line):
            return
        angle_data = (int(curr_line[2], 16) << 16 | int(curr_line[3], 16) << 8 | int(curr_line[1], 16))
        self.cal_sum += self._twos_complement(angle_data)
        self.cal_count += 1

    def _parse_fog_data(self, curr_line, prev_line):
        if not (self._handle_checksum(curr_line) and self._handle_checksum(prev_line)):
            return

        packet_count = int(curr_line[4], 16)
        angle_data = (int(curr_line[2], 16) << 16 | int(curr_line[3], 16) << 8 | int(curr_line[1], 16))
        self.angle_sum += (self._twos_complement(angle_data) - self.bias)
        self.count += 1

        if self.count >= self.samples:
            angle_mv = (self.angle_sum / self.samples) * (2.5 / (2**23))
            angle_deg_sec = angle_mv * self.integration_factor
            self.integrated_sum += angle_deg_sec * (time.time() - self.prev_time)
            self.parsed_data["angle_deg"] = self.integrated_sum
            self.publish_reading(self.integrated_sum)
            self.prev_time = time.time()
            self.angle_sum = 0
            self.count = 0

        if packet_count % 2 != 0 and packet_count < 8:
            xData = (int(prev_line[5], 16) << 8) | int(curr_line[5], 16)
            self.data[self.xDataNames[packet_count // 2]] = xData

        self._translate_data()

    def _translate_data(self):
        if "temp" in self.data:
            self.parsed_data["temp"] = self.data["temp"] * (250 / (2**15)) - 50
        if "supply_voltage" in self.data:
            self.parsed_data["supply_voltage"] = self.data["supply_voltage"] * (10 / (2**15))
        if "sld_curr" in self.data:
            self.parsed_data["sld_curr"] = self.data["sld_curr"] * (0.25 / (2**15))
        if "diag_sig" in self.data:
            self.parsed_data["diag_sig"] = self.data["diag_sig"] * (2.5 / (2**15))

    def _read_fog(self, callback=None):
        line = []
        prev_line = []
        while self.readData:
            while self.ser.in_waiting:
                if not self.readData:
                    break
                byte = self.ser.read(1)
                if byte == b'\xdd':
                    if len(line) == 8 and (not prev_line or prev_line[4] != line[4]):
                        (callback or self._parse_fog_data)(line, prev_line)
                        prev_line = line
                    line = [byte.hex()]
                else:
                    line.append(byte.hex())

    def publish_reading(self, reading):
        msg = Float64()
        msg.data = reading
        self.pub_fog_ang.publish(msg)

    def stop_read(self):
        self.readData = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join()
        print("FOG read thread stopped.")

    def close(self):
        if self.readData:
            self.stop_read()
        self.ser.close()
        print("FOG serial port closed.")


if __name__ == "__main__":
    fog = FOG(deviceHelper.dataFromConfig("fog"))
    fog.calibrate()
    rospy.spin()