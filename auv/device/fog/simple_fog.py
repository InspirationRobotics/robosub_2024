import time
import serial

from auv.utils import deviceHelper

class SimpleFOG:
    def __init__(self, port='/dev/ttyUSB0'):
        """Initialize the serial connection"""
        self.ser = self._setupSerial(port)
    
    def _setupSerial(self, p: str) -> serial.Serial:
        """Sets up the serial connection."""
        return serial.Serial(
            port=p, 
            baudrate=921600, 
            parity=serial.PARITY_NONE, 
            stopbits=serial.STOPBITS_ONE, 
            bytesize=serial.EIGHTBITS)
    
    def _read_fog(self):
        """Read the FOG sensor data continuously."""
        line = []
        
        while True:
            if self.ser.in_waiting:
                # Read one byte from the serial connection
                byte = self.ser.read(1)

                # Append the byte to the current line
                if byte == b'\xdd':
                    if len(line) == 8:
                        # When a complete line of 8 bytes is read
                        # Print the raw angle data (3 bytes corresponding to the angle)
                        angle_data = int(line[1], 16) << 16 | int(line[2], 16) << 8 | int(line[3], 16)
                        print(f"Raw Angle Data: {angle_data}")
                    line = [byte.hex()]
                else:
                    line.append(byte.hex())

if __name__ == "__main__":
    fog_port = deviceHelper.dataFromConfig("fog")

    fog = SimpleFOG(fog_port)  # Change the port if necessary

    print("Reading raw FOG data...")
    try:
        fog._read_fog()  # Continuously read and print the raw angular velocity data
    except KeyboardInterrupt:
        print("\nStopping FOG data reading...")
        fog.ser.close()
        print("FOG serial port closed.")
