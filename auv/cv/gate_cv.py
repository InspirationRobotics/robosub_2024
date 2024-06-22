"""
CV code for the gate mission. 

Should be relatively straightforward: get the detection data from the ML model, then align with the midpoint of the given detection, assuming the 
confidence of the detection is high enough.
"""

# Import what you need from within the package.

import time

import cv2
import numpy as np

class CV:
    """
    CV class for the Gate mission. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.

    Attributes:
        self.shape (tuple): (height, width) of the frame.
    """

    # Camera to get the camera stream from.
    camera = "/auv/camera/videoOAKdRawForward" 
    model = "gate"

    def __init__(self, **config):
        """
        Initialize the CV class. 
        Setup/attributes here will contain everything needed for the run function.
        
        Args:
            config: Dictionary that contains the configuration of the devices on the sub.
        """

        self.shape = (480, 640)

        print("[INFO] Gate CV init")

    def run(self, frame, target, detections):
        """
        Run the CV script.

        Args:
            frame: The frame from the camera stream
            target (str): The side of the gate to choose, either blue or red. 
            detections (list): This only applies to OAK-D cameras; this is the list of detections from the ML model output

        Here should be all the code required to run the CV.
        This could be a loop, grabbing frames using ROS, etc.

        Returns:
            dictionary, visualized frame: {lateral motion command, forward motion command, yaw motion command, end flag}, visualized frame
        """
        print("[INFO] Gate CV run")

        forward = 0
        lateral = 0
        yaw = 0
        end = 0

        for detection in detections:
            print(f"Detection label: {detection.label}")
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": end}, frame
