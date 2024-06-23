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

        self.config = config
        self.shape = (640, 480)
        self.end = False

        self.state = None
        self.detected = False
        self.tolerance = 20 # Pixels

        print("[INFO] Gate CV init")

    def yaw_smart(self, detection):
        """Determine yaw either clockwise or counterclockwise if there is one detection on the screen, based on where the detection is on the screen."""

        detection_midpoint = (detection.xmin + detection.xmax)/2
        midpoint_frame = self.shape[0]/2

        # If detection is to the left of the center of the frame.
        if detection_midpoint < midpoint_frame - self.tolerance: 
            yaw = -1
        # If detection is to the right of the center of the frame.
        elif detection_midpoint > midpoint_frame + self.tolerance:
            yaw = 1
        else:
            yaw = 1

        return yaw
    
    def strafe_smart(self, detection_x):
        """Strafe to align with the correct side of the gate based on target x_coordinate."""

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

        # If there are zero detections, yaw
        # If there is one detection, note on which side of the screen it is and then yaw accordingly (offsource to a different function)
        # If there are two detections, check confidences and label, then begin strafe.
        # Once aligned, end.

        if len(detections) == 0:
            yaw = 1
        elif len(detections) == 1:
            detection = detections[0]
            yaw = self.yaw_smart(detection)
        elif len(detections) == 2:
            for detection in detections:
                x_midpoint = (detection.xmin + detection.xmax)/2 
                if detection.confidence > 0.5 and target in detection.label:
                    target_x = x_midpoint
                elif detection.confidence > 0.5 and target not in detection.label:
                    other_x = x_midpoint
                else:
                    print(f"[WARN] Detections have low confidence, going for the highest confidence label.")
        


            
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": self.end}, frame
