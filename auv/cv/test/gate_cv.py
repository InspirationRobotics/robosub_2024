import cv2
import time
import numpy as np

import os

class CV:
    def __init__(self, config):
        self.aligned = False
        self.shape = (640, 480)
        self.detected = True
        self.config = config # Blue counterclockwise, Red clockwise
        self.step = None

        # Test variables.
        self.detection_area = None

    def detect_red(self, frame):
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
    
    def detect_blue(self, frame):
        """
        Uses HSV color space and masking to detect a red object.
        """
        detected = False
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_blue_mask = np.array([100, 150, 0])
        upper_blue_mask = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue_mask, upper_blue_mask)

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
        

    def run(self, raw_frame):
        visualized_frame = None
        forward = 0
        lateral = 0
        yaw = 0
        x_tolerance = 20 # Pixels

        # TODO: Create run logic

if __name__ == "__main__":
    video_root_path = "/home/kc/Desktop/Team Inspiration/RoboSub 2024/Training Data/"
    mission_name = "Gate/"
    video_name = "Gate Video 1.mp4"
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

                # motion_values, viz_frame, detection_status, detection_coords = cv.run(frame)
                # if viz_frame is not None:
                #     cv2.imshow("frame", viz_frame)

                if frame is not None:
                    cv2.imshow("frame", frame)
                else:
                    print("[ERROR] Unable to display frame.")

                # print(f"Motion: {motion_values}, Detection status: {detection_status}, Detection Coords: {detection_coords}")
                # print(f"Step: {cv.step}, Area of detection {cv.detection_area}")
                time.sleep(0.05)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break