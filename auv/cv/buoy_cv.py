"""
Buoy CV. Finds the buoy, locks into the buoy, and navigates to the correct position in order to perform circumnavigation.
"""

import cv2
import time
import numpy as np

import os


class CV:
    camera = "/auv/camera/videoUSBRaw0" 

    def __init__(self, **config):
        self.aligned = False
        self.shape = (640, 480)
        self.x_midpoint = self.shape[0] / 2
        self.y_midpoint = self.shape[1] / 2
        self.frame_area = self.shape[0] * self.shape[1]
        self.tolerance = 50 # Pixels

        self.detected = False
        self.prev_detected = False
        self.config = config # Blue counterclockwise, Red clockwise
        self.step = None

        self.end = False

        # Sets yaw magnitude. Due to camera latency, this needs to decrease
        # when the buoy gets off the screen
        self.search_yaw = 1
        self.pass_count = 0
        self.depth_time = time.time()

        # Test variables.
        self.detection_area = None

        print("[INFO] Buoy CV init")

    def detect_buoy(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255])) + \
               cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 0:
                x, y, w, h = cv2.boundingRect(largest_contour)
                # ymin is top left corner of bounding box, ymax is bottom right
                return {"status": True, "xmin": x, "xmax": x + w, "ymin": y, "ymax": y + h}, frame
        return {"status": False, "xmin": None, "xmax": None, "ymin": None, "ymax": None}, frame
    
    def movement_calculation(self, detection):
        """Calculates the movement that the sub should use based on the detection -- detection is a dictionary containing the 
        bounding box coordinates (everything that is returned by detect_buoy)"""

        forward = 0
        lateral = 0
        yaw = 0
        vertical = 0
        
        # Detect the buoy
        if detection.get("status") == True:
            # Find pixel area of buoy bounding box
            buoy_area = abs(detection.get("xmax") - detection.get("xmin")) * abs(detection.get("ymin") - detection.get("ymax"))

            # Filter false positives
            if buoy_area < 100:
                self.detected = False
                self.step = None
                yaw = 1
            else:
                self.detected = True
                self.step = 1
        else:
            self.detected = False
            self.step = None
            yaw = 1

        if self.detected == True:
            # Get x midpoint of buoy bounding box
            x_coordinate = (detection.get("xmin") + detection.get("xmax"))/2

            # Yaw to center-align AUV and buoy
            if x_coordinate < self.x_midpoint - self.tolerance:
                yaw = -0.65
            elif x_coordinate > self.x_midpoint + self.tolerance:
                yaw = 0.65
            else:
                yaw = 0

            # Adjust depth to be equal to buoy - ensures y minimum in bottom half
            # of image and y maximum in top half of image
            # See here: https://pyimagesearch.com/2021/01/20/opencv-getting-and-setting-pixels/

            # ymax is bottom right corner, ymin is top left corner
            if detection.get("ymin") > self.y_midpoint + self.tolerance:
                # Go down by 0.05 m - need to 
                # play around with depth functions
                # in water testing before coding this
                depth_param = 0.05
            elif detection.get("ymax") < self.y_midpoint - self.tolerance:
                # Go up by 0.05 m
                depth_param = -0.05
            else:
                depth_param = 0
            
            # Approach to a set distance from buoy

            if buoy_area < 14000: # number of pixels in buoy's bounding box
                forward = 2.5
            else:
                forward = 0
                self.end = True
            
            # Adjust vertical only if there is a depth parameter
            # and it has been at least 10 seconds since the last
            # adjustment

            if depth_param and (time.time() - self.depth_time > 10):
                self.depth_time = time.time()
                print(f"[DEBUG] ymin is {detection.get('ymin')} and ymax is {detection.get('ymax')}")
                vertical = depth_param

            # print(f"[INFO] Buoy area : {buoy_area}")

        return forward, lateral, yaw, vertical


    def run(self, raw_frame, target, detections):
        """Run the CV logic."""
        visualized_frame = None

        data_from_detection, frame = self.detect_buoy(raw_frame)

         
        if frame is not None:
            visualized_frame = frame
        else:
            visualized_frame = None

        forward, lateral, yaw, vertical = self.movement_calculation(data_from_detection)

        end = self.end

        return {"lateral" : lateral, "forward" : forward, "yaw" : yaw, "vertical": vertical, "end" : end}, visualized_frame
