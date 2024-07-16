import serial
import os
import time
import scipy.signal
from scipy.optimize import minimize

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

    def get_signal(self):
        #Voltage data from microphones
        mic1_vol = np.array([....])
        mic2_vol = np.array([....])
        mic3_vol = np.array([....])
        sampling_rate = 44100

        #Find the peaks of the signals
        peak1 = np.argmax(mic1_vol)
        peak2 = np.argmax(mic2_vol)
        peak3 = np.argmax(mic3_vol)

        #Convert peak indices to times
        time1 = peak1/sampling_rate
        time2 = peak2/sampling_rate
        time3 = peak3/sampling_rate

    #Step 2: Calculate TDOA

        tdoa_12 = time2 - time1
        tdoa_13 = time3 - time1

    #Step 3: Setup Coordinates

        mic1_coords = np.array([x1, y1]
        mic2_coords = np.array([x2, y2]))
        mic3_coords = np.array([x3, y3])

    #Step 4: Formulate and Solve Equations

        speed_of_sound = 1500 #m/s

    def equations(vars):
        x, y = vars
        d1 = np.sqrt((x - mic1_coords[0]**2) + (y - mic1_coords[1])**2)
        d2 = np.sqrt((x - mic2_coords[0]**2) + (y - mic2_coords[1])**2)
        d3 = np.sqrt((x - mic3_coords[0]**2) + (y - mic3_coords[1])**2)

        eq1 = (d2 - d1) - tdoa_12 * speed_of_sound
        eq2 = (d3 - d1) - tdoa_13 * speed_of_sound

        return eq1**2 + eq2**2

        #Intial Guess
        initial_guess = [0, 0]

        #Solve the system of equations
        result = minimize(equations, initial_guess)

        #source coordinates
        source_coords = result.x
        print(f"Estimated source coordinates: {source_coords}")
