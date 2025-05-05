import serial
import time
import sys
# https://github.com/ehavugi/vectornav-imu/blob/master/imu.py
# Open serial port
ser = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=1)  # adjust port as you needed

def read_imu(ser):
    """
    Reads IMU data from the serial port and returns a dictionary with keys:
    "Yaw", "Pitch", "Roll", "MagX", "MagY", "MagZ", "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ"
    """
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line.startswith("$VNYMR"):
        x = line.split(",")
        elements = ["Yaw", "Pitch", "Roll", "MagX", "MagY", "MagZ", 
                    "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ"]
        if len(x) >= 13:
            data = {}
            for i, key in enumerate(elements):
                value = x[i+1]
                if key == "GyroZ":
                    value = value.split("*")[0]  # Remove checksum
                data[key] = value
            return data
    return None

# Main loop
try:
    print("Reading IMU data at 10 Hz. Press Ctrl+C to stop.")
    while True:
        imu_data = read_imu(ser)
        if imu_data:
            output = ' | '.join(f'{k}: {v}' for k, v in imu_data.items())
            # Pad with spaces to overwrite old line
            sys.stdout.write(f'\r{output.ljust(120)}')
            sys.stdout.flush()
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    ser.close()
    print("Serial port closed.")
