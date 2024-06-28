"""
Path CV Test Class.

Author: Avika Prasad
"""

# Import what you need from within the package.

import time
import math
import cv2
import numpy as np
import os

class CV:
    """
    Path CV class. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """

    def __init__(self, config):
        self.aligned = False
        self.shape = (640, 480)
        self.detected = False
        self.config = config 
        self.step = 0

        self.end = False

        # Test variables.
        self.oriented = False
        self.aligned = False

        self.following_path = False
        
        self.lateral_search = False
        self.start_time = None
        self.lateral_time_search = 3 # Seconds
        self.last_lateral = 0

    def run(self, orig_frame):
        lateral = 0
        forward = 0
        yaw = 0
        end = self.end
        # Downscale the image to a reasonable size to reduce compute
        scale = 1

        # Minimize false detects by eliminating contours less than a percentage of the image
        area_threshold = 0.01
        croppedPixels = 150
        width = orig_frame.shape[0]
        height = orig_frame.shape[1] - croppedPixels
        dim = (int(scale * height), int(scale * width))

        orig_frame = cv2.resize(orig_frame, dim, interpolation=cv2.INTER_AREA)
        frame = cv2.GaussianBlur(orig_frame, (5, 5), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask0 = cv2.inRange(hsv, (0, 100, 50), (20, 255, 255))
        mask1 = cv2.inRange(hsv, (20, 100, 50), (25, 255, 255))
        # join masks
        mask = mask0 + mask1

        ret, thresh = cv2.threshold(mask, 127, 255, 0)
        # Erosions and dilations
        # erosions are applied to reduce the size of foreground objects
        kernel = np.ones((3, 3), np.uint8)
        eroded = cv2.erode(thresh, kernel, iterations=0)	
        dilated = cv2.dilate(eroded, kernel, iterations=3) 

        dst = cv2.equalizeHist(dilated)
        cv2.imshow("equalized", dst)

        cnts, hierarchy = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:60]

        boundingBoxes = np.empty((0, 4), float)
        if len(cnts) > 0:
            self.detected = True
            contour = cnts[0]

            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            cv2.drawContours(orig_frame, [box], 0, (0, 0, 255), 2)
            
            # initializing the points of the rectangle
            box0 = (box[0])

            box1 = (box[1])	

            box2 = (box[2])
                
            box3 = (box[3])
            
            #line for vertical path
            v_start_left = (int((box0[0] + box1[0]) / 2), int((box0[1] + box1[1]) / 2))
            v_end_right = (int((box2[0] + box3[0]) / 2), int((box2[1] + box3[1]) / 2))

            #line for horizantal path
            h_start_left = (int((box0[0] + box3[0]) / 2), int((box0[1] + box3[1]) / 2))
            h_end_right = (int((box1[0] + box2[0]) / 2), int((box1[1] + box2[1]) / 2))
            
            #calculate distance of the vertical line (using the distance formula)
            v_distance = math.sqrt((v_end_right[0] - v_start_left[0])**2 + (v_end_right[1] - v_start_left[1])**2)

            #calculate distance of the horizantal line (using the distance formula)
            h_distance = math.sqrt((h_end_right[0] - h_start_left[0])**2 + (h_end_right[1] - h_start_left[1])**2)

            #drawing the largest line
            if v_distance > h_distance:
                cv2.line(orig_frame, v_start_left, v_end_right, (0, 255, 0), 2)
                x1, y1 = v_start_left
                x2, y2 = v_end_right
                start_left = v_start_left
                end_right = v_end_right
            else:
                cv2.line(orig_frame, h_start_left, h_end_right, (0, 255, 0), 2)
                x1, y1 = h_start_left
                x2, y2 = h_end_right
                start_left = h_start_left
                end_right = h_end_right

            #calculating the slope
            if x2 - x1 != 0:
                slope = (y2 - y1)/(x2 - x1)

            else:
                slope = 10000
                print("Vertical line: infinite slope")
            
        else:
            self.detected = False

        #motion code
        if self.detected == False and self.following_path == False:
            self.lateral_search = True

        if self.detected == True:
            self.lateral_search == False

        if self.detected == False and self.following_path == True:
            self.end = True

        """
        First find the object.

        If not detected, execute lateral search pattern:
        - We cannot just execute search pattern if the path is not detected.
        - If the path has not been detected, we want to move back and forth laterally:
            - Specifically, move right 3 secs, then move left for slightly longer (4 secs ex.)
            - Keep moving right and left with slightly longer time iterations until the path has been detected.

        If detected, yaw to orient correctly with slope. 

        If oriented correctly, laterally move to align with center -- do this simultaneously with 
        continuous checking of slope.

        If aligned and oriented correctly, move forward. If after moving forward detection is gone, that means
        mission has been successful and we can end.

        """
        # Back and forth lateral pattern
        if self.lateral_search:
            if self.start_time is None:
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
            
                 
        # else: 
        #     # switch direction after the interval 
        #     if self.last_lateral == 1:
        #             lateral = -1
        #     elif self.last_lateral == -1:
        #             lateral = 1
        #         # Reset the start time for the next interval
        #     self.start_time = time.time()
        #         # Increment the interval time
        #     self.lateral_time_search += 1
        #     # Update last lateral
        #     self.last_lateral = lateral
        
        if self.detected:
            threshold_slope = 10
            if abs(slope) > threshold_slope:
                self.oriented = True
            else:
                self.oriented = False
                if slope > 0:
                    yaw = -1
                elif slope < 0:
                    yaw = 1
                else:
                    pass

            if self.oriented:
                x_threshold = 20 # Pixels
                center_x = self.shape[0]/2
                center_line_x = (start_left[0] + end_right[0])/2

                if abs(center_line_x - center_x) < x_threshold:
                    self.aligned = True
                    lateral = 0
                elif center_line_x - center_x > x_threshold:
                    self.aligned = False
                    lateral = 1 # Move right
                elif center_line_x - center_x < - x_threshold:
                    self.aligned = False
                    lateral = -1 # Move left

            if self.aligned and self.oriented:
                lateral = 0
                yaw = 0
                forward = 1
                self.following_path = True
            
            # Continuously return motion commands, the state of the mission, and the visualized frame.
        return {"lateral": lateral, "forward": forward, "yaw" : yaw, "end": end}, orig_frame

if __name__ == "__main__":
    video_root_path = "/home/kc/Desktop/Team Inspiration/RoboSub 2024/Training Data/"
    mission_name = "Follow the Path/"
    video_name = "Follow the Path Video 2.mp4"
    video_path = os.path.join(video_root_path, mission_name, video_name)
    print(f"Video path: {video_path}")

    cv = CV("Blue")

    if not os.path.exists(video_path):
        print(f"[ERROR] Video file not found {video_path}")
    else:
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"[ERROR] Unable to open video file: {video_path}")
        else:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("[INFO] End of file.")
                    break

                motion_values, viz_frame = cv.run(frame)

                if viz_frame is not None:
                    cv2.imshow("frame", viz_frame)
                else:
                    print("[ERROR] Unable to display frame.")
                
                print(f"Motion values: {motion_values}")
                time.sleep(0.05)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
