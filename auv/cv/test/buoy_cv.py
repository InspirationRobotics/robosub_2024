import cv2
import time
import numpy as np

import os

# Import the video 
# Read each frame
# Detect the buoy through color thresholding
# Return motion values

class CV:
    def __init__(self, config):
        self.aligned = False
        self.shape = (640, 480)
        self.detected = True
        self.config = config # Blue counterclockwise, Red clockwise
        self.step = None

        # Test variables.
        self.detection_area = None

    def detect_buoy(self, frame):
        """
        Uses HSV color space and masking to detect a red object.
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
            
            # else:
            #     return {"frame" : frame, "xmin" : None, "xmax" : None, "ymin" : None, "ymax" : None}

    def run(self, raw_frame):
        visualized_frame = None
        forward = 0
        lateral = 0
        yaw = 0
        x_tolerance = 20 # Pixels

        data_from_detection, frame = self.detect_buoy(raw_frame)

        if frame is not None:
            visualized_frame = frame
        else:
            visualized_frame = None

        x_midpoint = self.shape[0]/2
        y_midpoint = self.shape[1]/2

        # Check if there is a detection, if there is not one then yaw.

        if data_from_detection.get("status") == True:
            self.detected = True
            detection_midpoint = (data_from_detection.get('xmin') + data_from_detection.get('xmax'))/2
            detection_area = abs((data_from_detection.get('xmax') - data_from_detection.get('xmin')) * (data_from_detection.get('ymax') - data_from_detection.get('ymin')))
        else:
            self.detected = False

        # Make this more robust
        if self.detected == False:
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
        elif self.step == 1 and self.detected == True:
            self.detection_area = detection_area
            if detection_area < 1/60 * self.shape[0] * self.shape[1]:
                forward = 1
            else:
                forward = 0
                self.step = 2

        # Step 2 is align with offset.
        elif self.step == 2 and self.detected == True:
            x_offset_cw = x_midpoint/2
            x_offset_ccw = x_midpoint + x_midpoint/2

            if self.config == "Blue": # Counterclockwise -- we want the buoy on the right side of the frame
                if detection_midpoint > x_offset_ccw - x_tolerance:
                    self.aligned = True
                else:
                    lateral = 1
            elif self.config == "Red": # We want buoy on left side
                if detection_midpoint < x_offset_cw + x_tolerance:
                    self.aligned = True
                else: 
                    lateral = -1
        
        if self.aligned == True:
            self.step == 3
        
        # TODO: Everything up to Step 3 works well, so next is just finish the steps.
        elif self.step == 3 and self.detected == True:
            lateral = 0
            forward = 1
                
        
        return {"lateral" : lateral, "forward" : forward, "yaw" : yaw}, visualized_frame, self.detected, data_from_detection

if __name__ == "__main__":
    video_root_path = "/home/kc/Desktop/Team Inspiration/RoboSub 2024/Training Data/"
    mission_name = "Buoy/"
    video_name = "Buoy Video 5.mp4"
    video_path = os.path.join(video_root_path, mission_name, video_name)
    print(f"Video path: {video_path}")

    cv = CV("Blue")

    if not os.path.exists(video_path):
        print(f"[ERROR] Video file not found {video_path}")
    else:
        cap = cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"[ERROR] Unable to open video file: {video_path}")
        else:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("[INFO] End of file.")
                    break

                motion_values, viz_frame, detection_status, detection_coords = cv.run(frame)
                if viz_frame is not None:
                    cv2.imshow("frame", viz_frame)
                else:
                    print("[ERROR] Unable to display frame.")

                print(f"Motion: {motion_values}, Detection status: {detection_status}, Detection Coords: {detection_coords}")
                print(f"Step: {cv.step}, Area of detection {cv.detection_area}")
                time.sleep(0.05)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break