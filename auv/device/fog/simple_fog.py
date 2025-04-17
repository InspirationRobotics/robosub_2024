import time
import serial
import csv
from auv.utils import deviceHelper

class SimpleFOG:
    # Reference voltage for the FOG sensor
    def __init__(self, port='/dev/ttyUSB0', csv_file='fog_data.csv'):
        """Initialize the serial connection and CSV file for logging"""
        self.ser = self._setupSerial(port)
        self.reference_voltage = 5
        self.csv_file = csv_file
        self._initialize_csv()

    def _setupSerial(self, p: str) -> serial.Serial:
        """Sets up the serial connection."""
        return serial.Serial(
            port=p, 
            baudrate=921600, 
            parity=serial.PARITY_NONE, 
            stopbits=serial.STOPBITS_ONE, 
            bytesize=serial.EIGHTBITS)
    
    def _initialize_csv(self):
        """Initialize the CSV file with headers"""
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'Raw Angle Data', 'Voltage Data (mV)'])
    
    def _read_fog(self):
        """Read the FOG sensor data continuously and log to CSV"""
        line = []
        
        while True:
            if self.ser.in_waiting:
                # Read one byte from the serial connection
                byte = self.ser.read(1)

                # Append the byte to the current line
                if byte == b'\xdd':
                    if len(line) == 8:
                        # When a complete line of 8 bytes is read
                        angle_data = int(line[1], 16) << 16 | int(line[2], 16) << 8 | int(line[3], 16)
                        raw_value = int(line[5], 16) << 8 | int(line[6], 16)
                        voltage_data = (raw_value / (2**15)) * self.reference_voltage * 1000

                        # Get the current timestamp
                        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

                        # Log the data to CSV
                        self._log_to_csv(timestamp, angle_data, voltage_data)

                        # Print the logged data
                        print(f"Timestamp: {timestamp}")
                        print(f"Raw Angle Data: {angle_data}")
                        print(f"Voltage Data: {voltage_data} mV")

                    # Start a new line
                    line = [byte.hex()]
                else:
                    line.append(byte.hex())

    def _log_to_csv(self, timestamp, angle_data, voltage_data):
        """Logs the data to the CSV file"""
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, angle_data, voltage_data])

if __name__ == "__main__":
    fog_port = deviceHelper.dataFromConfig("fog")

    fog = SimpleFOG(fog_port)  # Change the port if necessary

    print("Reading raw FOG data and logging to CSV...")
    try:
        fog._read_fog()  # Continuously read and log the raw angular velocity and voltage data
    except KeyboardInterrupt:
        print("\nStopping FOG data reading...")
        fog.ser.close()
        print("FOG serial port closed.")
