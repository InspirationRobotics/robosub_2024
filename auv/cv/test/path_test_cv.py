"""
Template script for creating CV logic for a specific mission, and testing it on training data.

Author: Avika Prasad
"""

import cv2
import time
import numpy as np

import os

class CV:
    """CV class, DO NOT change the name of the class."""

    def __init__(self, config):
        # Config is a way of passing in an argument to indicate to the logic what actions to take. Take a look at 
        # buoy_test_cv.py for an example.
        self.shape = (640, 480)

        # Switcher variables which can be used as needed to switch states.
        self.aligned = False
        self.detected = False

        self.config = config 
        self.step = 1 # Step counter variable.

        self.end = False # End variable to denote when the mission has finished.

        # Add variables as needed below.

    # You can put detection functions to detect a specific object as needed. 

    def detect_path(self, frame):
        detected = False
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_orange_mask = np.array([10, 100, 100])
        upper_orange_mask = np.array([25, 255, 255])

        mask = cv2.inRange(frame, lower_orange_mask, upper_orange_mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(hsv, contours, -1, (0, 255, 0), 3)

        # cv2.imshow('hsv', image)
        # cv2.waitKey(0)
        # cv2.destoryAllWindows()

       # if contours:
           # largest_contour = max(contours, key = cv2.contourArea)

            #if cv2.contourArea(largest_contour) > 0:
                #x, y, w, h = cv2.boundingRect(largest_contour)
               # cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                #detected = True
                #return {"status": detected, "xmin" : x, "xmax" : (x + w), "ymin" : (y), "ymax" : (y + h)}, hsv
        
        return {"status": detected, "xmin" : None, "xmax" : None, "ymin" : None, "ymax" : None}, frame 
    
    def align_properly(self, detection):
        # Detection a list/dictionary containing the detection coordinates
        # Align properly based on when the path is straight up and down the screen.
        # Yaw so that path is |, then lateral to midpoint, then yaw again.
        pass

    def run(self, frame):
        """ Run the CV logic. Returns the motion commands and visualized frame. """
        # converts the frame from color to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

         # converts the color values for BGR to HSV
        into_hsv =(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV))

         # lowest limit to say it is orange
        lower_limit=np.array([25, 41, 98])
         # 3, 20, 20 

         # highest limit to say it is orange
        upper_limit=np.array([23, 100, 82])
         # 80, 255, 255

        kernel = np.ones((5, 5), np.uint8)

        # takes the frame and turns the pixels in the limit to white (255), outside of limit = black (0)
        orange=cv2.inRange(into_hsv,lower_limit,upper_limit)

        # determines the edges by determining which are stong and weak
        edges = cv2.Canny(gray, threshold1=100, threshold2=200)

        ret, thresh3 = cv2.threshold(edges, 230, 255, cv2.THRESH_BINARY)
        ret2, thresh3 = cv2.threshold(thresh3, 1, 255, cv2.THRESH_OTSU)
        cv2.imshow("edges", thresh3)
         # Removing Noise
        orange = cv2.morphologyEx(orange, cv2.MORPH_OPEN, kernel)
        time.sleep(0.2)
         # Blur and Threshold 
        orange = cv2.GaussianBlur(orange, (11, 11), 0)
        ret, thresh = cv2.threshold(orange, 230, 255, cv2.THRESH_BINARY)

        blur = cv2.blur(thresh, (10, 10))
        ret2, thresh2 = cv2.threshold(blur, 1, 255, cv2.THRESH_OTSU)

         # Find Contours
        contours, heirarchy = cv2.findContours(thresh3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, 255, 3)

        yaw = lateral = forward = 0
        hline = cv2.line(frame, (640, 0), (0, 0), (0, 255, 255), 8)

        lateral = 0
        forward = 0
        yaw = 0

        center_line = (hline)/2

        if center_line < 5:
            lateral = 3

        # path_data, frame = self.detect_path(raw_frame)

        # if path_data.get('status') == True:
        #     self.detected = True
        #     detection_xmin = path_data.get('xmin')
        #     detection_xmax = path_data.get('xmax')
        #     detection_ymin = path_data.get('ymin')
        #     detection_ymax = path_data.get('ymax')

        if self.step == 0 and self.detected == True:
            pass

        if self.step == 1 and self.detected == True:
            forward = 1

        # Step 0: align with the path correctly
        # Step 1: Move forward along the path -- do so in a way that if you move off the path, you'll
        # move back to step 0 and align properly again.

        # Return the frame and the motion values.
        return {"lateral" : lateral, "forward" : forward, "yaw" : yaw}, frame

# This if statement is just saying what to do if this script is run directly. 
if __name__ == "__main__":
    # Example of how to obtain a training video. Make sure to follow this template when capturing your own video, in case 
    # another team member needs to run this code on his/her device. 
    
    # NOTE: When downloading the training data, the training data folder itself, which contains all of the data.
    video_root_path = "/Users/avikaprasad/Desktop/RoboSub 2024/Training Data" # Computer path through the training data folder.
    mission_name = "Follow the Path/" # Mission folder
    video_name = "Follow the Path Video 4.mp4" # Specified video
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
                motion_values, viz_frame = cv.run(frame)
                if viz_frame is not None:
                    cv2.imshow("frame", viz_frame)
                else:
                    print("[ERROR] Unable to display frame.")

                # For testing purposes.
                print(f"Motion: {motion_values}")
                
                time.sleep(0.05)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
