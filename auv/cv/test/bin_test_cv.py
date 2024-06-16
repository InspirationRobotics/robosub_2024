"""Bin Mission CV Test"""

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
        self.shape = (640, 480)
        self.aligned = False
        self.detected = False
        self.config = config 
        self.step = 0
        self.end = False

        print("[INFO] Bin CV Init")
        self.viz_frame = None
        self.error_buffer = []

    def get_bbox_center(self, detection):
        x1 = detection["xmin"]
        x2 = detection["xmax"]
        y1 = detection["ymin"]
        y2 = detection["ymax"]

        return ((x1 + x2) // 2, (y1 + y2) // 2)

    def detect_red(self, frame):
        detected = False
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red_mask1 = np.array([0, 120, 70])
        upper_red_mask1 = np.array([10, 255, 255])
        lower_red_mask2 = np.array([170, 120, 70])
        upper_red_mask2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red_mask1, upper_red_mask1)
        mask2 = cv2.inRange(hsv, lower_red_mask2, upper_red_mask2)
        mask = mask1 + mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > 0:
                x, y, w, h = cv2.boundingRect(largest_contour)
                detected = True
                return {"status": detected, "xmin": x, "xmax": x + w, "ymin": y, "ymax": y + h}

        return {"status": detected, "xmin": None, "xmax": None, "ymin": None, "ymax": None}

    def detect_blue(self, frame):
        detected = False
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_blue_mask = np.array([100, 50, 50])
        upper_blue_mask = np.array([130, 255, 255])        
        mask = cv2.inRange(hsv, lower_blue_mask, upper_blue_mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > 0:
                x, y, w, h = cv2.boundingRect(largest_contour)
                detected = True
                return {"status": detected, "xmin": x, "xmax": x + w, "ymin": y, "ymax": y + h}
        
        return {"status": detected, "xmin": None, "xmax": None, "ymin": None, "ymax": None}
    
    def get_midpoints(self, frame):
        """
        Detects both red and blue rectangles in the frame and returns their midpoints.
        """
        blue_info = self.detect_blue(frame)
        red_info = self.detect_red(frame)
        midpoints = {}

        if blue_info["status"]:
            blue_xmin = blue_info["xmin"]
            blue_xmax = blue_info["xmax"]
            blue_ymin = blue_info["ymin"]
            blue_ymax = blue_info["ymax"]
            cv2.rectangle(frame, (blue_xmin, blue_ymin), (blue_xmax, blue_ymax), (255, 0, 0), 2)
            midpoints["blue"] = self.get_bbox_center(blue_info)

        if red_info["status"]:
            red_xmin = red_info["xmin"]
            red_xmax = red_info["xmax"]
            red_ymin = red_info["ymin"]
            red_ymax = red_info["ymax"]
            cv2.rectangle(frame, (red_xmin, red_ymin), (red_xmax, red_ymax), (0, 0, 255), 2)
            midpoints["red"] = self.get_bbox_center(red_info)

        return midpoints
    
    def run(self, raw_frame):
        return self.get_midpoints(raw_frame)
    
if __name__ == "__main__":
    video_root_path = "/Users/brandontran3/downloads/Training Data/"
    mission_name = "Bins/"
    video_name = "Bins Video 3.mp4"
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

                midpoints = cv.run(frame)
                if midpoints:
                    print(f"Midpoints: {midpoints}")
                    cv2.imshow("frame", frame)
                else:
                    print("[ERROR] Unable to display frame.")

                time.sleep(0.05)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            cap.release()
            cv2.destroyAllWindows()
