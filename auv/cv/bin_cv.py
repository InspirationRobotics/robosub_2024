"""
Bin CV. Locates the correct side of the bin (red or blue), and aligns with the intention to drop the marker into the correct side of the bin.
"""

# Import what you need from within the package.

import time

import cv2
import numpy as np

class CV:
    """
    Bin CV class. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """

    # Camera to get the camera stream from.
    camera = "/auv/camera/videoOAKdRawBottom"
    model = "bins" 

    def __init__(self, **config):
        """
        Initialize the CV class. 
        Setup/attributes here will contain everything needed for the run function.
        
        Args:
            config: Dictionary that contains the configuration of the devices on the sub.
        """

        # This is an example of getting a stored value from the configuration. 
        self.config = config
        self.shape = (640, 480)
        self.state = "search"

        self.x_midpoint = self.shape[0]/2
        self.y_midpoint = self.shape[1]/2

        self.start_time = None
        self.last_lateral = 0
        self.lateral_time_search = 2

        self.depth_time = time.time()

        self.tolerance = 20 # Pixels
        self.aligned = False
        self.end = False

        print("[INFO] Bin CV init")

    def smart_movement(self, detection_x, detection_y):
        """Function to determine the correct strafe and forward values to align with the detection"""
        if detection_x < self.x_midpoint - self.tolerance:
            lateral = -0.5
        elif detection_x > self.x_midpoint + self.tolerance:
            lateral = 0.5
        else:
            lateral = 0

        # NOTE: Y goes from top down, so top pixel will be 1, bottom will be 480
        if detection_y < self.y_midpoint - self.tolerance:
            forward = 0.5
        elif detection_y > self.y_midpoint + self.tolerance:
            forward = -0.5
        else:
            forward = 0

        return forward, lateral

    def run(self, frame, target, detections):
        """
        Run the CV script.

        Args:
            frame: The frame from the camera stream
            target: This can be any type of information, for example, the object to look for
            detections: This only applies to OAK-D cameras; this is the list of detections from the ML model output

        Here should be all the code required to run the CV.
        This could be a loop, grabbing frames using ROS, etc.

        Returns:
            dictionary, visualized frame: {motion commands/flags for servos and other indication flags}, visualized frame
        """

        lateral = 0
        forward = 0
        yaw = 0
        vertical = 0

        # Find the number of detections, if no detections, enter a search grid
        # If there is one detection, move up to increase field of view
        # If there are two detections, lock into the right detection

        if len(detections) == 0:
            self.state = "search"

        elif len(detections) == 1:
            self.state = "up"

        elif len(detections) == 2:
            self.state = "lock"
            for detection in detections:
                x_midpoint = (detection.xmin + detection.xmax)/2
                y_midpoint = (detection.ymin + detection.ymax)/2
                if detection.confidence > 0.65 and target in detection.label:
                    target_x = x_midpoint
                    target_y = y_midpoint
                elif detection.confidence > 0.65 and target not in detection.label:
                    other_x = x_midpoint
                    other_y = y_midpoint

        if self.state == "search":
            forward = 0.5
            if self.start_time == None:
                self.start_time = time.time()
                self.last_lateral = 1  # Initial direction

            elapsed_time = time.time() - self.start_time

            if elapsed_time < self.lateral_time_search:
                lateral = self.last_lateral
            else:
                # Switch direction and reset timer
                self.last_lateral = -self.last_lateral
                self.start_time = time.time()
                lateral = self.last_lateral
                self.lateral_time_search += 1

        if self.state == "up":
            if time.time() - self.depth_time > 10:
                self.depth_time = time.time()
                vertical = 0.05

        if self.state == "lock":
            if target_x and target_y:
                forward, lateral = self.smart_movement(target_x, target_y)
                if forward == 0 and lateral == 0:
                    self.aligned = True
            else:
                print("[ERROR] Unable to detect the target")

        if self.aligned == True:
            self.end = True

        # Continuously return motion commands, the state of the mission, and the visualized frame.
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "vertical": vertical, "end": self.end}, frame
