"""
Buoy CV.

Author: Keith Chen
"""

import cv2
import time
import numpy as np

import os


class CV:
    camera = "/auv/camera/videoOAKdRawForward" 

    def __init__(self, **config):
        self.aligned = False
        self.shape = (640, 480)
        self.midpoint = self.shape[0] / 2
        self.frame_area = self.shape[0] * self.shape[1]
        self.tolerance = 25 # Pixels

        self.detected = False
        self.prev_detected = False
        self.config = config # Blue counterclockwise, Red clockwise
        self.step = None

        self.end = False

        # Sets yaw magnitude. Due to camera latency, this needs to decrease
        # when the buoy gets off the screen
        self.search_yaw = 1
        self.yaw_mag = 0.3
        self.pass_count = 0
        self.prev_time = time.time()

        # Test variables.
        self.detection_area = None

    def detect_buoy(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # print("Timestamp for hsv variable: ", time.time() - self.prev_time)
        mask = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255])) + \
               cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        # print("Timestamp for mask variable: ", time.time() - self.prev_time)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # print("Timestamp for contours variable: ", time.time() - self.prev_time)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            # print("Timestamp for largest_contour variable: ", time.time() - self.prev_time)
            if cv2.contourArea(largest_contour) > 0:
                x, y, w, h = cv2.boundingRect(largest_contour)
                # print("Timestamp for return: ", time.time() - self.prev_time)
                return {"status": True, "xmin": x, "xmax": x + w, "ymin": y, "ymax": y + h}, frame
        # print("Timestamp for return: ", time.time() - self.prev_time)
        return {"status": False, "xmin": None, "xmax": None, "ymin": None, "ymax": None}, frame
        
    
    def movement_calculation(self, detection):
        """TODO: Split into search and approach portions"""
        forward = 0
        lateral = 0
        yaw = 0
        
        # Detect the buoy
        if detection.get("status") == True:
            # Find pixel area of buoy bounding box
            buoy_area = abs(detection.get("xmax") - detection.get("xmin")) * abs(detection.get("ymin") - detection.get("ymax"))

            # Filter false positives
            if buoy_area < 250:
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
            x_coordinate = (detection.get("xmin") + detection.get("xmax"))/2
            if x_coordinate < self.midpoint - self.tolerance:
                yaw = -1
            elif x_coordinate < self.midpoint + self.tolerance:
                yaw = 1
            else:
                yaw = 0

            if buoy_area < 10000:
                forward = 1.0
            elif buoy_area < 15000: # number of pixels in buoy's bounding box
                forward = 0.5
            elif buoy_area > 20000:
                forward = -0.7
            else:
                forward = 0
                if yaw == 0:
                    self.end = True
            
            print(f"[INFO] Frame area : {self.frame_area}")
            print(f"[INFO] Buoy area : {buoy_area}")

        return forward, lateral, yaw


    def run(self, raw_frame, target, detections):
        """Run the CV logic."""
        visualized_frame = None

        self.prev_time = time.time()
        data_from_detection, frame = self.detect_buoy(raw_frame)

         
        if frame is not None:
            visualized_frame = frame
        else:
            visualized_frame = None

        forward, lateral, yaw = self.movement_calculation(data_from_detection)

        end = self.end

        return {"lateral" : lateral, "forward" : forward, "yaw" : yaw, "end" : end}, visualized_frame
