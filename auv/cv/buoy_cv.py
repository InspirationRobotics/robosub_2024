# """
# CV code for the gate mission. 

# Should be relatively straightforward: get the detection data from the ML model, then align with the midpoint of the given detection, assuming the 
# confidence of the detection is high enough.
# """

# # Import what you need from within the package.

# import time

# import cv2
# import numpy as np

# class CV:
#     """
#     CV class for the Gate mission. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.

#     Attributes:
#         self.shape (tuple): (height, width) of the frame.
#     """

#     # Camera to get the camera stream from.
#     camera = "/auv/camera/videoOAKdRawForward" 
#     model = "buoy"

#     def __init__(self, **config):
#         """
#         Initialize the CV class. 
#         Setup/attributes here will contain everything needed for the run function.
        
#         Args:
#             config: Dictionary that contains the configuration of the devices on the sub.
#         """

#         self.shape = (480, 640)

#         print("[INFO] Buoy CV init")

#     def run(self, frame, target, detections):
#         """
#         Run the CV script.

#         Args:
#             frame: The frame from the camera stream
#             target (str): The side of the gate to choose, either blue or red. 
#             detections (list): This only applies to OAK-D cameras; this is the list of detections from the ML model output

#         Here should be all the code required to run the CV.
#         This could be a loop, grabbing frames using ROS, etc.

#         Returns:
#             dictionary, visualized frame: {lateral motion command, forward motion command, yaw motion command, end flag}, visualized frame
#         """
#         print("[INFO] Buoy CV run")

#         forward = 0
#         lateral = 0
#         yaw = 0
#         end = False

#         for detection in detections:
#             detection_type = type(detection)
#             detection_label = detection.label
#             print(f"Detection type: {detection_type}, Detection label: {detection_label}")
#             print(f"Detection coords: ({detection.xmin, detection.ymin}) , ({detection.xmax, detection.ymax})")
        

#         # Continuously return motion commands, the state of the mission, and the visualized frame.
#         return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": end}, frame

# if __name__ == "__main__":
#     cv = CV()
#     data, frame = cv.run(None, None, None)
#     print(data)

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
        self.detected = False
        self.config = config # Blue counterclockwise, Red clockwise
        self.step = 0

        self.forward_times = 0 # The theory behind this is that we should only go forward past the buoy twice.
        self.time_before_forward_run = 0
        self.before_end_lateral_time = 0
        self.end = False

        # Test variables.
        self.detection_area = None

    def detect_buoy(self, frame):
        """
        Uses HSV color space and masking to detect a red object. Returns bounding box coordinates and the visualized frame.
        """
        detected = False
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red_mask_1 = np.array([0, 120, 70])
        upper_red_mask_1 = np.array([10, 255, 255])
        lower_red_range_mask = cv2.inRange(hsv, lower_red_mask_1, upper_red_mask_1)

        lower_red_mask_2 = np.array([170, 120, 70])
        upper_red_mask_2 = np.array([180, 255, 255])
        upper_red_range_mask = cv2.inRange(hsv, lower_red_mask_2, upper_red_mask_2)

        mask = lower_red_range_mask + upper_red_range_mask

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key = cv2.contourArea)

            if cv2.contourArea(largest_contour) > 0:
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                detected = True
                return {"status": detected, "xmin" : x, "xmax" : (x + w), "ymin" : (y), "ymax" : (y + h)}, frame
        
        return {"status": detected, "xmin" : None, "xmax" : None, "ymin" : None, "ymax" : None}, frame        

    def run(self, raw_frame, target, detections):
        """Run the CV logic."""
        visualized_frame = None
        forward = 0
        lateral = 0
        yaw = 0
        end = self.end
        x_tolerance = 20 # Pixels
        if raw_frame:
            print("[INFO] Raw frame exists.")
        data_from_detection, frame = self.detect_buoy(raw_frame)

        if frame is not None:
            visualized_frame = frame
            print("[INFO] Visualized frame exists.")
        else:
            visualized_frame = None

        x_midpoint = self.shape[0]/2
        y_midpoint = self.shape[1]/2

        x_offset_ccw = x_midpoint/2
        x_offset_cw = x_midpoint + x_midpoint/2

        # Check if there is a detection, if there is not one then yaw.

        if data_from_detection.get("status") == True:
            self.detected = True
            detection_midpoint = (data_from_detection.get('xmin') + data_from_detection.get('xmax'))/2
            detection_area = abs((data_from_detection.get('xmax') - data_from_detection.get('xmin')) * (data_from_detection.get('ymax') - data_from_detection.get('ymin')))
        else:
            self.detected = False

        if self.detected == False and self.step is None:
            yaw = 1
            forward = 0
            lateral = 0

        if self.detected == True and self.step is None:
            self.step = 0

        # We will use a sort of square method, where we will maneuver laterally to get the buoy on a certain part of the 
        # frame, then move forward.

        # Step 0 is align laterally with the center of the buoy
        if self.step == 0 and self.detected == True:
            if detection_midpoint > x_midpoint + x_tolerance: # Means too far to the right
                lateral = -1
            elif detection_midpoint < x_midpoint - x_tolerance:
                lateral = 1
            else: # Aligned
                lateral = 0
                self.step = 1
        
        # Step 1 is to move forward enough so that the buoy is at a certain area size
        if self.step == 1 and self.detected == True:
            self.detection_area = detection_area
            if detection_area < 1/60 * self.shape[0] * self.shape[1]:
                forward = 1
            else:
                forward = 0
                self.step = 2

        # Step 2 is align with offset.
        if self.step == 2 and self.detected == True:
            self.aligned = False

            if self.config == "Blue": # Counterclockwise -- we want the buoy on the left side of the frame
                if detection_midpoint < x_offset_ccw + x_tolerance:
                    self.aligned = True
                    self.step = 3
                else:
                    lateral = 1
            elif self.config == "Red": # We want buoy on right side
                if detection_midpoint > x_offset_cw - x_tolerance:
                    self.aligned = True
                    self.step = 3
                else: 
                    lateral = -1

        if self.aligned == False:
            self.time_before_forward_run = time.time()

        forward_allotment = 3 # Seconds
        
        if self.step == 3:
            lateral = 0
            if time.time() - self.time_before_forward_run < forward_allotment:
                forward = 2
            else:
                forward = 0
                self.forward_times += 1
                self.aligned = False
                self.step = 4

        # Here we stop, and yaw, then go back to step 2.
        if self.step == 4:
            # We'll want to yaw 180 degrees -- since we can't use a gyroscope/don't want to rely on it, 
            # instead we will do the opposite of step 2, where basically we will yaw until we find the buoy 
            # on the side of the frame that we don't want it on, so we can go back to step 2 and move laterally.

            if self.config == "Blue": # Counterclockwise -- we want the buoy on the LEFT side of the frame
                if self.detected == True:
                    if detection_midpoint > x_offset_cw - x_tolerance:
                        self.aligned = True
                        self.step = 2
                    else:
                        yaw = -1
                else:
                    yaw = -1
            elif self.config == "Red": # We want buoy on RIGHT side
                if self.detected == True:
                    if detection_midpoint < x_offset_ccw + x_tolerance:
                        self.aligned = True
                        self.step = 2
                    else:
                        yaw = 1
                else:
                    yaw = 1

        if self.forward_times < 2:
            self.before_end_lateral_time = time.time()
        
        # This is to make sure we get something equivalent to a full circle/square instead of unfinished.
        if self.forward_times == 2:
            lateral_end_allotment = 2 # Again in seconds
            if self.config == "Blue":
                # We'll (hopefully) be on the left of the buoy
                if time.time() - self.before_end_lateral_time < lateral_end_allotment:
                    lateral = -1
                else:
                    end = True
            if self.config == "Red":
                # Right side of buoy
                if time.time() - self.before_end_lateral_time < lateral_end_allotment:
                    lateral = 1
                else:
                    end = True

        # We only need dictionary and visualized_frame, self.detected and data_from_detection are test variables.
        return {"lateral" : lateral, "forward" : forward, "yaw" : yaw, "end" : end}, visualized_frame

        # We only need dictionary and visualized_frame, self.detected and data_from_detection are test variables.
        return {"lateral" : lateral, "forward" : forward, "yaw" : yaw, "end" : end}, visualized_frame
