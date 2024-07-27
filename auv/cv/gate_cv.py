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
        self.aligned = False
        self.tolerance = 20 # Pixels

        self.target = None
        self.force_target = True

        print("[INFO] Gate CV init")
    
    def strafe_smart(self, detection_x):
        """Strafe to align with the correct side of the gate based on target x_coordinate."""
        midpoint_frame = self.shape[0]/2
        # If detection is to the left of the center of the frame.
        if detection_x < midpoint_frame - self.tolerance: 
            lateral = -1
        # If detection is to the right of the center of the frame.
        elif detection_x > midpoint_frame + self.tolerance:
            lateral = 1
        else:
            lateral = 0

        return lateral
    
    def detection_area(self, detection):
        return ((detection.xmax - detection.xmin) * (detection.ymax - detection.ymin))

    def run(self, frame, target="Blue", detections=None):
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
        # print("[INFO] Gate CV run")

        forward = 0
        lateral = 0
        yaw = 0
        
        target_x = None
        other_x = None

        # If there are zero detections, yaw.
        # If there is one detection, note on which side of the screen it is and then yaw accordingly (offsource to a different function).
        # If there are two detections, check confidences and label, then begin strafe.
        # Once aligned, end.

        if len(detections) == 0:
            yaw = 1
            self.state = None
        elif len(detections) >= 1:
            for detection in detections:
                x_midpoint = (detection.xmin + detection.xmax)/2
                print(f"[DEBUG]: Detection confidence is {detection.confidence}") 
                if detection.confidence > 0.8 and target in detection.label:
                    target_x = x_midpoint
                    self.target = detection.label
                    self.state = "strafe"
                elif detection.confidence > 0.8 and target not in detection.label:
                    other_x = x_midpoint
                    other_label = detection.label
                elif detection.confidence >= 0.5:
                    print(f"[WARN] Detections have low confidence, going for the highest confidence label.")
                    self.state = "approach"

            if target_x == None and other_x != None:
                if self.force_target:
                    print("[INFO] Continuing search for target")
                    yaw = 1
                else:
                    print("[INFO] Switching targets because original set target is not confirmed.")
                    target_x = other_x
                    self.target = other_label
                    self.state = "strafe"

        if self.state == "strafe":
            yaw = 0 # Just in case not already 0
            lateral = self.strafe_smart(target_x)
            if lateral == 0:
                self.area = self.detection_area(detection)
                if self.area < 1000 or self.area > 1500:
                    self.state = "approach"
                else:
                    self.aligned = True
        
        if self.state == "approach":
            confidence = 0
            self.area = self.detection_area(detection)
            if self.area < 5000:
                print("[INFO] Moving forward.")
                forward = 1
            elif self.area > 7500:
                print("[INFO] Moving backward.")
                forward = -1
            else:
                self.state == "strafe" # Strafe anyway
            for detection in detections:
                if detection.confidence > confidence:
                    target_x = (detection.xmin + detection.xmax) / 2
                    confidence = detection.confidence
                    self.target = detection.label


        if self.aligned == True:
            self.end = True
            
        print(f"[INFO] State : {self.state}")
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "target": self.target, "end": self.end}, frame
