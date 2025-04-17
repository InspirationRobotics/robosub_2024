import time
import serial
import csv
from auv.utils import deviceHelper

class SimpleFOG:
    def __init__(self, port='/dev/ttyUSB0', csv_file='fog_data.csv'):
        """Initialize the serial connection and CSV file for logging"""
        self.ser = self._setupSerial(port)
        self.reference_voltage = 5
        self.csv_file = csv_file
        self._initialize_csv()

        # Integration-related variables
        self.angle_sum = 0
        self.count = 0
        self.samples = 10
        self.bias = 0  # Set your bias if known
        self.integration_factor = 1.0  # Set based on SF
        self.integrated_sum = 0
        self.prev_time = time.time()

    def _setupSerial(self, p: str) -> serial.Serial:
        return serial.Serial(
            port=p, 
            baudrate=921600, 
            parity=serial.PARITY_NONE, 
            stopbits=serial.STOPBITS_ONE, 
            bytesize=serial.EIGHTBITS)

    def _initialize_csv(self):
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'Raw Angle Data', 'Voltage Data (mV)', 'Integrated Angle (deg)'])

    def _twos_complement(self, val, bits=24):
        if val & (1 << (bits - 1)):
            val -= 1 << bits
        return val

    def _handle_checksum(self, line):
        # Dummy checksum function for now (implement if needed)
        return True

    def _parse_fog_data(self, curr_line, prev_line):
        if not self._handle_checksum(curr_line) or not self._handle_checksum(prev_line):
            return None

        angle_data = int(curr_line[2], 16) << 16 | int(curr_line[3], 16) << 8 | int(curr_line[1], 16)
        angle_signed = self._twos_complement(angle_data)
        self.angle_sum += (angle_signed - self.bias)
        self.count += 1

        Xdata = int(curr_line[5], 16)
        if self.count >= self.samples:
            angle_mv = (self.angle_sum / self.samples) * (2.5 / (2**23)) * 1000  # mV
            angle_deg_sec = angle_mv * self.integration_factor
            dt = time.time() - self.prev_time
            self.integrated_sum += angle_deg_sec * dt

            self.prev_time = time.time()
            self.angle_sum = 0
            self.count = 0

            return angle_deg_sec, Xdata, self.integrated_sum
        return None

    def _read_fog(self):
        line = []
        prev_line = None
        cnt = 0
        while True:
            if self.ser.in_waiting:
                byte = self.ser.read(1)
                if byte == b'\xdd':
                    if len(line) == 8:
                        print(f"Raw 8-byte packet: {line}")  # 🔍 Debug print
                        curr_line = line
                        parsed = self._parse_fog_data(curr_line, prev_line)
                        prev_line = curr_line
                        if parsed:
                            angle_data, voltage_data, integrated_angle = parsed
                            timestamp = time.strftime('%Y-%m-%d %H:%M:%S') + f"_{cnt}"
                            self._log_to_csv(timestamp, angle_data, voltage_data, integrated_angle)
                            print(f"Timestamp: {timestamp}")
                            print(f"Raw Angle Data: {angle_data}")
                            print(f"Voltage Data: {voltage_data:.2f} mV")
                            print(f"Integrated Angle: {integrated_angle:.2f} deg")
                    line = [byte.hex()]
                else:
                    line.append(byte.hex())

            time.sleep(1/60)
            cnt += 1

    def _log_to_csv(self,  timestamp, angle_data, voltage_data, integrated_angle):
            with open(self.csv_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, angle_data, voltage_data, integrated_angle])

if __name__ == "__main__":
    fog_port = deviceHelper.dataFromConfig("fog")
    fog = SimpleFOG(fog_port)
    print("Reading FOG data and logging to CSV...")
    try:
        fog._read_fog()
    except KeyboardInterrupt:
        print("\nStopping FOG data reading...")
        fog.ser.close()
        print("FOG serial port closed.")