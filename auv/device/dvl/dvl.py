# Documentation for Teledyne available at https://drive.google.com/file/d/1xniDtjYIJhaFOhWaSj4tj6RxBdN5inx2/view
# See page 93 for specs

import math
import signal
import threading
import time
import json

import numpy as np

import serial

from . import dvl_tcp_parser
from auv.utils import deviceHelper


class DVL:
    """DVL class to enable position estimation"""

    def __init__(self, autostart=True, compass=False, test=False):
        self.test = test
        if not self.test:
            self.dvlPort = deviceHelper.dataFromConfig("dvl")
            print(self.dvlPort)
            sub = deviceHelper.variables.get("sub")
            print(f"[DEBUG] Sub is {sub}")
            if sub == "onyx":
                self.ser = serial.Serial(
                    port=self.dvlPort,
                    baudrate=115200,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                )

                self.dvl_rot = math.radians(45)
                self.ser.isOpen()
                self.ser.reset_input_buffer()
                self.ser.send_break()
                time.sleep(1)
                startPing = "CS"
                self.ser.write(startPing.encode())
                time.sleep(2)
                self.read = self.read_onyx

            elif sub == "graey":
                # autostart = False
                self.read = self.read_graey
                self.dvl_rot = math.radians(0)
            else:
                raise ValueError(f"Invalid sub {sub}")

        self.enable_compass = compass

        self.__running = False
        self.__thread_vel = None
        self.prev_time = None
        self.current_time = None
        
        # sensor error
        self.compass_error = math.radians(1.0)  # rad/s
        
        self.error = [0, 0, 0]  # accumulated error

        # NORTH = 0, EAST = pi/2, SOUTH = pi, WEST = 3pi/2
        self.compass_rad = None  # rad

        self.vel_rot = [0, 0, 0]  # rotated velocity vector

        # position in meters
        # X = lateral
        # Y = forward
        # Z = {"Onyx": "distance from bottom",
        # "Graey": "Distance from surface"}
        self.position = [0, 0, 0]  


        self.is_valid = False
        self.data_available = False

        # stores position and error history for context manager
        self.position_memory = []
        self.error_memory = []

        # DVL data packet. Must put this in __init__ to allow for the
        # time to accumulate in read_graey
        self.graey_data = {
            "time": 0,  # seconds
            "vx": 0,  # m/s
            "vy": 0,  # m/s
            "vz": 0,  # m/s
            "error": 0, # m/s - is placeholder for dynamic value
            "valid": False,  # boolean
            }

        if autostart:
            self.start()

    def __parseLine(self, line):
        """Parse line"""
        return line.decode("utf-8").replace(" ", "").replace("\r\n", "").split(",")

    def read_graey(self):
        """Get velocity from graey"""

        # Useful documentation for JSON strings: 
        # https://www.geeksforgeeks.org/convert-json-to-dictionary-in-python/

        # On Graey run python3 -m (filepath) velocity -i 192.168.2.10
        try:
            data_iterator = dvl_tcp_parser.main()
            for line in data_iterator:
                line = json.loads(line)

                # In Graey, x-axis is forward and y-axis is lateral,
                # to be consistent w/ Onyx we will switch them here
                self.graey_data["time"] += float(line["time"]) / 1000
                self.graey_data["vx"] = float(line["vy"])
                self.graey_data["vy"] = float(line["vx"])
                self.graey_data["vz"] = float(line["vz"])
                self.graey_data["error"] = float(line["fom"])
                self.graey_data["valid"] = line["velocity_valid"]
                return self.graey_data
        except:
            print("I threw an exception!")
            data = None
        return data

    def read_onyx(self):
        """Get velocity from onyx"""

        while not self.ser.in_waiting:
            # take a nap :)
            time.sleep(0.01)
            # print("[DEBUG] Serial is not working!!!")

        data = {
            "time": 0,  # seconds
            "vx": 0,  # m/s
            "vy": 0,  # m/s
            "vz": 0,  # m/s
            "error": 0.002, # m/s - see Teledyne Documentation note above
            "valid": False,  # boolean
        }
        SA = self.__parseLine(self.ser.readline())
        if SA[0] != ":SA":
            return None

        TS = self.__parseLine(self.ser.readline())
        WI = self.__parseLine(self.ser.readline()) # unused
        BI = self.__parseLine(self.ser.readline())
        WS = self.__parseLine(self.ser.readline()) # unused
        BS = self.__parseLine(self.ser.readline())
        WE = self.__parseLine(self.ser.readline()) # unused
        BE = self.__parseLine(self.ser.readline())  # unused
        WD = self.__parseLine(self.ser.readline()) # unused
        BD = self.__parseLine(self.ser.readline())


        try:
            # data["Attitude"] = [float(SA[1]), float(SA[2]), float(SA[3])]
            # data["Salinity"] = float(TS[2])
            # data["Temp"] = float(TS[3])
            # data["Transducer_depth"] = float(TS[4])
            # data["Speed_of_sound"] = float(TS[5])
            # data["Result_code"] = TS[6]
            # data["DVL_velocity"] = [int(BI[1]), int(BI[2]), int(BI[3]), int(BI[4])]
            # data["isDVL_velocity_valid"] = BI[5] == "A"
            # data["AUV_velocity"] = [int(BS[1]), int(BS[2]), int(BS[3])]
            # data["isAUV_velocity_valid"] = BS[4] == "A"
            # data["Distance_from_bottom"] = float(BD[4])
            # data["Time_since_valid"] = float(BD[5])

            centi = int(TS[1][12:14]) * 0.01
            seconds = int(TS[1][10:12])
            minutes = int(TS[1][8:10]) * 60
            hours = int(TS[1][6:8]) * 60 * 60
            t = hours + minutes + seconds + centi

            # this is the only data we need
            data["time"] = t
            data["vx"] = int(BS[1]) / 1000
            data["vy"] = int(BS[2]) / 1000
            data["vz"] = int(BS[3]) / 1000
            data["valid"] = BS[4] == "A"
        except:
            print("I failed")
            data = None
        # print("[DEBUG] Data in read_onyx: ", data)
        return data

    def process_packet_compass(self, packet):
        """integrate velocity into position"""

        vel = [packet.get("vx", 0), packet.get("vy", 0), packet.get("vz", 0)]
        self.current_time = packet.get("time", 0)  # seconds
        self.dvl_error = packet.get("error", 0)

        if self.prev_time is None or self.compass_rad is None:
            self.prev_time = self.current_time
            print("[WARN] DVL not ready, waiting for compass or some more sample")
            return False

        dt = self.current_time - self.prev_time
        if dt < 0:
            print("[WARN] DVL time error, skipping")
            return False

        self.is_valid = packet["valid"]
        if not self.is_valid:
            # print("[WARN] DVL velocity not valid, skipping")
            return False

        self.prev_time = self.current_time

        # rotate velocity vector using compass heading
        # plus 45 degrees
        # X = lateral, Y = forward, Z = vertical
        self.vel_rot = [
            vel[0] * math.cos(self.compass_rad + self.dvl_rot) + vel[1] * math.sin(self.compass_rad + self.dvl_rot),
            vel[1] * math.cos(self.compass_rad + self.dvl_rot) - vel[0] * math.sin(self.compass_rad + self.dvl_rot),
            vel[2],
        ]

        # integrate velocity to position with respect to time
        self.position = [
            self.position[0] + self.vel_rot[0] * dt,
            self.position[1] + self.vel_rot[1] * dt,
            self.position[2] + self.vel_rot[2] * dt,
        ]

        vel_rot_error = [
            (vel[0] + self.dvl_error) * math.cos(self.compass_rad + self.dvl_rot + self.compass_error)
            + (vel[1] + self.dvl_error) * math.sin(self.compass_rad + self.dvl_rot + self.compass_error),
            (vel[1] + self.dvl_error) * math.cos(self.compass_rad + self.dvl_rot + self.compass_error)
            - (vel[0] + self.dvl_error) * math.sin(self.compass_rad + self.dvl_rot + self.compass_error),
            vel[2] + self.dvl_error,  # we actually have a sensor for depth, so useless
        ]

        # calculate accumulated error
        self.error = [
            self.error[0] + abs(self.vel_rot[0] - vel_rot_error[0]) * dt,
            self.error[1] + abs(self.vel_rot[1] - vel_rot_error[1]) * dt,
            self.error[2] + abs(self.vel_rot[2] - vel_rot_error[2]) * dt,
        ]

        return True

    def process_packet(self, packet):
        """Integrate velocity into position without compass"""

        vel = [packet.get("vx", 0), packet.get("vy", 0), packet.get("vz", 0)]
        self.current_time = packet.get("time", 0)  # seconds
        self.dvl_error = packet.get("error", 0)

        if self.prev_time is None:
            self.prev_time = self.current_time
            print("[WARN] DVL not ready, waiting for some more sample")
            return False

        dt = self.current_time - self.prev_time
        if dt < 0:
            print("[WARN] DVL time error, skipping")
            return False

        self.is_valid = packet["valid"]
        if not self.is_valid:
            # print("[WARN] DVL velocity not valid, skipping")
            return False
        
        # print("[DEBUG] Running the process packet method")
        # print(f"[DEBUG]: Current time is {self.current_time}")
        # print(f"[DEBUG]: Previous time is {self.prev_time}")

        self.prev_time = self.current_time

        # Rotate velocity vector by DVL rotation of 45 degrees

        self.vel_rot = [
            vel[0] * math.cos(self.dvl_rot) + vel[1] * math.sin(self.dvl_rot),
            vel[1] * math.cos(self.dvl_rot) - vel[0] * math.sin(self.dvl_rot),
            vel[2],
        ]

        # integrate velocity to position with respect to time
        self.position = [
            self.position[0] + self.vel_rot[0] * dt,
            self.position[1] + self.vel_rot[1] * dt,
            self.position[2] + self.vel_rot[2] * dt,
        ]

        vel_error = [
            vel[0] + self.dvl_error,
            vel[1] + self.dvl_error,
            vel[2] + self.dvl_error,
        ]

        # calculate accumulated error
        self.error = [
            self.error[0] + abs(vel[0] - vel_error[0]) * dt,
            self.error[1] + abs(vel[1] - vel_error[1]) * dt,
            self.error[2] + abs(vel[2] - vel_error[2]) * dt,
        ]

        return True

    def reset_position(self):
        """Reset position to 0"""
        self.position = [0, 0, 0]
        self.error = [0, 0, 0]

    def update(self):
        """Update DVL data (runs in a thread)"""
        print("[DEBUG] Called update()")
        while self.__running:
            vel_packet = self.read()
            if vel_packet is None:
                continue
            if self.enable_compass:
                ret = self.process_packet_compass(vel_packet)
            else:
                ret = self.process_packet(vel_packet)
            self.data_available = ret

    def start(self):
        # ensure not running
        print("[DEBUG] Started successfully")
        if self.__running:
            print("[WARN] DVL already running")
            return

        self.__running = True
        self.__thread_vel = threading.Thread(target=self.update, daemon=True)
        self.__thread_vel.start()

    def stop(self):
        self.__running = False
        self.__thread_vel.join()

    def __enter__(self):
        """Context manager for DVL"""
        if not self.__running and not self.test:
            self.start()

        # begin a context, store current position
        self.position_memory.append(self.position)
        self.error_memory.append(self.error)

        # reset position
        self.reset_position()

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Exit context manager"""
        prev_pos = self.position_memory.pop()
        prev_error = self.error_memory.pop()

        # restore previous position and error
        self.position = [
            self.position[0] + prev_pos[0],
            self.position[1] + prev_pos[1],
            self.position[2] + prev_pos[2],
        ]

        self.error = [
            self.error[0] + prev_error[0],
            self.error[1] + prev_error[1],
            self.error[2] + prev_error[2],
        ]

if __name__ == '__main__':
    # Make a new dvl instance
    dvl1 = DVL()
    # while dvl1.current_time == None:
    #     time.sleep(0.01)
    # prev_time = dvl1.current_time
    while True:
        time.sleep(1.0)
        # print("[DEBUG: Ran a check on DVL timing]")
        print(dvl1.position)
        # prev_time = dvl1.current_time
        # print(dvl1.error)
