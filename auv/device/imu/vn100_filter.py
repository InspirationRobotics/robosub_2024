import time
import threading
from serial import Serial
import numpy as np
from scipy.signal import iirdesign, sosfiltfilt
from collections import deque
import csv
from datetime import datetime

# === FILTER SETTINGS (match MATLAB) ===
SAMPLE_RATE = 40      # Hz
FP = 5                # Hz passband edge
FSB = 7.5             # Hz stopband edge
GPASS = 1             # dB passband ripple
GSTOP = 60            # dB stopband attenuation
BUF_LEN = 160         # Rolling buffer size for filtering (4s at 40Hz)

# Design lowpass Butterworth (match MATLAB)
wp = FP / (SAMPLE_RATE / 2)
ws = FSB / (SAMPLE_RATE / 2)
sos = iirdesign(wp, ws, GPASS, GSTOP, ftype='butter', output='sos')

class VN100:
    def _init_(self, port: str = "COM3"):
        self.__port = port
        self.__bps = 115200
        self._ser = Serial(port=self.port, baudrate=self._bps, timeout=1)

        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.accX = 0.0
        self.accY = 0.0
        self.accZ = 0.0
        self.gyroX = 0.0
        self.gyroY = 0.0
        self.gyroZ = 0.0

        self.yaw_buf = deque(maxlen=BUF_LEN)
        self.pitch_buf = deque(maxlen=BUF_LEN)
        self.roll_buf = deque(maxlen=BUF_LEN)
        self.lock = threading.Lock()

        self.read_thread = threading.Thread(target=self.read, daemon=True)
        self.read_thread.start()
        time.sleep(2) # Give buffer time to fill

    def read(self):
        while True:
            time.sleep(1 / SAMPLE_RATE)
            try:
                data_line = self.__ser.readline().decode()
                data_list = data_line.split(',')

                yaw = (float(data_list[1]) + 90) % 360
                pitch = float(data_list[3])
                roll = float(data_list[2])

                with self.lock:
                    self.yaw = yaw
                    self.pitch = pitch
                    self.roll = roll
                    self.yaw_buf.append(yaw)
                    self.pitch_buf.append(pitch)
                    self.roll_buf.append(roll)

                    self.accX = float(data_list[4])
                    self.accY = float(data_list[5])
                    self.accZ = float(data_list[6])
                    self.gyroX = float(data_list[7])
                    self.gyroY = float(data_list[8])
                    self.gyroZ = float(data_list[9])

            except Exception:
                pass

    def get_filtered_ypr(self):
        with self.lock:
            # Only filter when buffer is long enough
            if len(self.yaw_buf) > 6:
                y = sosfiltfilt(sos, np.array(self.yaw_buf))
                p = sosfiltfilt(sos, np.array(self.pitch_buf))
                r = sosfiltfilt(sos, np.array(self.roll_buf))
                return y[-1], p[-1], r[-1]
            else:
                return self.yaw, self.pitch, self.roll

if _name_ == "_main_":
    sensor = VN100()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"data_{timestamp}.csv"

    data = []
    print("Recording... Press Ctrl+C to stop.")
    fieldnames = [
        "timestamp",
        "Raw_Yaw", "Raw_Pitch", "Raw_Roll",
        "Filt_Yaw", "Filt_Pitch", "Filt_Roll",
        "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ"
    ]

    try:
        while True:
            time.sleep(1 / SAMPLE_RATE)
            raw_yaw, raw_pitch, raw_roll = sensor.yaw, sensor.pitch, sensor.roll
            filt_yaw, filt_pitch, filt_roll = sensor.get_filtered_ypr()
            print(
                f"RAW YPR: ({raw_yaw:.2f}, {raw_pitch:.2f}, {raw_roll:.2f}) | "
                f"FILTERED YPR: ({filt_yaw:.2f}, {filt_pitch:.2f}, {filt_roll:.2f})"
            )
            data.append({
                "timestamp": datetime.now().isoformat(),
                "Raw_Yaw": raw_yaw,
                "Raw_Pitch": raw_pitch,
                "Raw_Roll": raw_roll,
                "Filt_Yaw": filt_yaw,
                "Filt_Pitch": filt_pitch,
                "Filt_Roll": filt_roll,
                "AccX": sensor.accX,
                "AccY": sensor.accY,
                "AccZ": sensor.accZ,
                "GyroX": sensor.gyroX,
                "GyroY": sensor.gyroY,
                "GyroZ": sensor.gyroZ
            })

    except KeyboardInterrupt:
        print("\nExiting and saving data to CSV...")
        with open(filename, mode="w", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(data)
        print(f"Data saved to {filename}")
    except Exception as e:
        print(f"Generic exception caught: {e}")