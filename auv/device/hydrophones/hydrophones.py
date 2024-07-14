import serial
import os
import time

from ..utils.deviceHelper import dataFromConfig
from auv.utils.deviceHelper import dataFromConfig

#hydrophone --> hydrophone pcb --> teensy --> computer (jetson)

class Hydrophones:
    def __init__(self):
        #intialize the usb port 
        self.usb = serial.Serial(port=dataFromConfig('teensy'))
    
    def read_teensy(self):
        if self.usb.in_waiting > 0:
            line = self.usb.readline().decode('utf-8').strip()
            v1, v2, v3 = map(int, line.split(', '))
