"""
CV for the gate mission. Finds the gate and aligns with the red or blue side of the gate, moving forward once aligned.
This one immediately goes forward through gate after alignment and will require tuning for how far to go
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
    model = "gate_woollett"

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
        self.tolerance = 80 # Pixels

        self.start_time = None
        self.last_yaw = 0
        self.prev_detected = False

        self.target = None
        self.force_target = False
        self.strafed = False

        print("[INFO] Gate CV init")
 
    def strafe_smart(self, detection_x):
        """Strafe to align with the correct side of the gate based on target x_coordinate."""
        if not self.strafed:
            self.strafed = True
            self.strafe_time = time.time()
        
        if time.time() - self.strafe_time > 5:
            lateral = 0
            self.end = True
            return lateral
        
        midpoint_frame = self.shape[0]/2

        # If detection is to the left of the center of the frame.
        if detection_x < midpoint_frame - self.tolerance: 
            lateral = -0.75
        # If detection is to the right of the center of the frame.
        elif detection_x > midpoint_frame + self.tolerance:
            lateral = 0.75
        else:
            lateral = 0

        return lateral
    
    def detection_area(self, detection):
        return ((detection.xmax - detection.xmin) * (detection.ymax - detection.ymin))

    def run(self, frame, target="Red", detections=None):
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

        if len(detections) == 0 and self.prev_detected == False:
            self.state = 'search'
        elif len(detections) == 0 and self.prev_detected == True:
            self.aligned = True
            self.end = True
        elif len(detections) >= 1:
            print("[DEBUG] Detection found")
            for detection in detections:
                x_midpoint = (detection.xmin + detection.xmax)/2
                print(f"[DEBUG] Detection confidence is {detection.confidence}") 
                print(f"[DEBUG] Detection label is {detection.label}")
                if detection.confidence > 0.5 and target in detection.label:
                    print(f"[DEBUG] Detected correct target with correct confidence")
                    self.prev_detected = True
                    target_x = x_midpoint
                    self.target = detection.label
                    self.state = "strafe"
                elif detection.confidence > 0.5 and target not in detection.label:
                    other_x = x_midpoint
                    other_label = detection.label
                # elif detection.confidence >= 0.5:
                #     self.state = "approach"

            if target_x == None and other_x != None:
                if self.force_target:
                    print("[DEBUG] Continuing search for target")
                    self.state = "search"
                else:
                    # print("[INFO] Switching targets because original set target is not confirmed.")
                    target_x = other_x
                    self.target = other_label
                    self.state = "strafe"
        
        if self.state == "search":
            # Scrap the yaw back and forth in favor of a simple clockwise search
            yaw = 1

        if self.state == "strafe":
            lateral = self.strafe_smart(target_x)
            if lateral == 0:
                self.state = "approach"
        
        if self.state == "approach":
            self.prev_detected = True
            if detection is not None:
                self.area = self.detection_area(detection)
            else:
                self.area = 1000000
            if self.area < 10000:
                forward = 2.5
                self.end = True
            else:
                self.aligned = True

        if self.aligned == True:
            self.end = True
            
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "target": self.target, "end": self.end}, frame
