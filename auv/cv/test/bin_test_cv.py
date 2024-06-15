"""Bin mission CV code"""

import time
import cv2
import numpy as np

import os

class CV: 
    """
    CV class for Bin mission. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """
  
    camera = "/auv/camera/videoOAKdRawBottom"
    model = "bin"
    
    def __init__(self, config):
        # Config is a way of passing in an argument to indicate to the logic what actions to take. Take a look at 
        # buoy_test_cv.py for an example.
        self.shape = (640, 480)

        # Switcher variables which can be used as needed to switch states.
        self.aligned = False
        self.detected = False

        self.config = config 
        self.step = 0 # Step counter variable.

        self.end = False # End variable to denote when the mission has finished.

        print("[INFO] Bin CV Init")
        
        # Add variables as needed below.
        self.viz_frame = None
        self.error_buffer = []
      
        print("[INFO] Bin CV Init")

    def get_bbox_center(self, detection):
        x1 = int(detection.xmin)
        x2 = int(detection.xmax)
        y1 = int(detection.ymin)
        y2 = int(detection.ymax)

        return ((x1 + x2) // 2, (y1 + y2) // 2)


    def detect_red(self, frame):
        """
        Uses HSV color space and masking to detect a red object.
        """
        detected = False
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for red color in HSV
        lower_red_mask1 = np.array([0, 120, 70])
        upper_red_mask1 = np.array([10, 255, 255])
        lower_red_mask2 = np.array([170, 120, 70])
        upper_red_mask2 = np.array([180, 255, 255])

        # Create masks for the red ranges
        mask1 = cv2.inRange(hsv, lower_red_mask1, upper_red_mask1)
        mask2 = cv2.inRange(hsv, lower_red_mask2, upper_red_mask2)

        # Combine masks
        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > 0:
                x, y, w, h = cv2.boundingRect(largest_contour)
                detected = True
                return {"status": detected, "xmin": x, "xmax": x + w, "ymin": y, "ymax": y + h}

        return {"status": detected, "xmin": None, "xmax": None, "ymin": None, "ymax": None}


    def detect_blue(self, frame):
        """
        Uses HSV color space and masking to detect a blue object.
        """
        detected = False
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_blue_mask = np.array([100, 50, 50])
        upper_blue_mask = np.array([130, 255, 255])        
        mask = cv2.inRange(hsv, lower_blue_mask, upper_blue_mask)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key = cv2.contourArea)

            if cv2.contourArea(largest_contour) > 0:
                x, y, w, h = cv2.boundingRect(largest_contour)
                detected = True
                return {"status": detected, "xmin" : x, "xmax" : (x + w), "ymin" : (y), "ymax" : (y + h)}
        
        return {"status": detected, "xmin" : None, "xmax" : None, "ymin" : None, "ymax" : None}
    
    def run(self, raw_frame):
        forward = 0
        lateral = 0
        yaw = 0

        height, width, _ = frame.shape

        tolerance = 0.1
        maxConfidence = 0
        target_color = None
        red = None
        blue = None
        red_confidence = 0
        blue_confidence = 0

        blue_info = self.detect_blue(raw_frame)
        red_info = self.detect_red(raw_frame)

        if blue_info.get('status') == True:
            blue_xmin = blue_info.get('xmin')
            blue_xmax = blue_info.get('xmax')
            blue_ymin = blue_info.get('ymin')
            blue_ymax = blue_info.get('ymax')
            cv2.rectangle(raw_frame, (blue_xmin, blue_ymin), (blue_xmax, blue_ymax), (255, 0, 0), 2)

        if red_info.get('status') == True:
            red_xmin = red_info.get('xmin')
            red_xmax = red_info.get('xmax')
            red_ymin = red_info.get('ymin')
            red_ymax = red_info.get('ymax')
            cv2.rectangle(raw_frame, (red_xmin, red_ymin), (red_xmax, red_ymax), (0, 0, 255), 2)

        return raw_frame
    
# This if statement is just saying what to do if this script is run directly. 
if __name__ == "__main__":
    # Example of how to obtain a training video. Make sure to follow this template when capturing your own video, in case 
    # another team member needs to run this code on his/her device. 
    
    # NOTE: When downloading the training data, the training data folder itself, which contains all of the data.
    video_root_path = "C:/Users/brand/OneDrive/Desktop/Training Data/" # Computer path through the training data folder.
    mission_name = "Bins/"
    video_name = "Bins Video 3.mp4" # Specified video
    video_path = os.path.join(video_root_path, mission_name, video_name)

    # For testing
    print(f"Video path: {video_path}")

    # Initialize an instance of the class.
    cv = CV("Blue")

    # Verify the path exists.
    if not os.path.exists(video_path):
        print(f"[ERROR] Video file not found {video_path}")
    else:
        # Capture the video object (basically access the specified video) at the specified path.
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"[ERROR] Unable to open video file: {video_path}")
        else:
            while True:
                # Access each frame of the video.
                ret, frame = cap.read()
                if not ret:
                    print("[INFO] End of file.")
                    break

                # Run the run function on the frame, and get back the relevant results.
                viz_frame = cv.run(frame)
                if viz_frame is not None:
                    cv2.imshow("frame", viz_frame)
                else:
                    print("[ERROR] Unable to display frame.")

                # For testing purposes.
                # print(f"Motion: {motion_values}")
                
                time.sleep(0.05)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
