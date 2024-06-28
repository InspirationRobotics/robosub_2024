"""
Bin CV and logic test.

Author: Brandon Tran
"""

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
        """
        Initialize the CV class with configuration settings.
        """
        self.shape = (640, 480)  # Set the frame shape
        self.aligned = False
        self.detected = False
        self.config = config.lower()  # Ensure config is in lowercase
        self.step = 0
        self.end = False
        self.drop_ball = False

        print(f"[INFO] Bin CV Init with target color: {self.config.capitalize()}")
        self.viz_frame = None
        self.error_buffer = []

        self.threshold = 0.1

    def get_bbox_center(self, detection):
        """
        Calculate the center of a bounding box.
        """
        x1 = detection["xmin"]
        x2 = detection["xmax"]
        y1 = detection["ymin"]
        y2 = detection["ymax"]
        return ((x1 + x2) // 2, (y1 + y2) // 2)

    def detect_red(self, frame):
        """
        Detect red objects in the frame and return their bounding box.
        """
        detected = False
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert frame to HSV color space

        # Define red color range in HSV
        lower_red_mask1 = np.array([0, 120, 70])
        upper_red_mask1 = np.array([10, 255, 255])
        lower_red_mask2 = np.array([170, 120, 70])
        upper_red_mask2 = np.array([180, 255, 255])

        # Create masks to detect red color
        mask1 = cv2.inRange(hsv, lower_red_mask1, upper_red_mask1)
        mask2 = cv2.inRange(hsv, lower_red_mask2, upper_red_mask2)
        mask = mask1 + mask2

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)  # Get the largest contour

            if cv2.contourArea(largest_contour) > 0:
                x, y, w, h = cv2.boundingRect(largest_contour)  # Get bounding box for the largest contour
                detected = True
                return {"status": detected, "xmin": x, "xmax": x + w, "ymin": y, "ymax": y + h}

        return {"status": detected, "xmin": None, "xmax": None, "ymin": None, "ymax": None}

    def detect_blue(self, frame):
        """
        Detect blue objects in the frame and return their bounding box.
        """
        detected = False
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert frame to HSV color space

        # Define blue color range in HSV
        lower_blue_mask = np.array([100, 50, 50])
        upper_blue_mask = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue_mask, upper_blue_mask)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)  # Get the largest contour

            if cv2.contourArea(largest_contour) > 0:
                x, y, w, h = cv2.boundingRect(largest_contour)  # Get bounding box for the largest contour
                detected = True
                return {"status": detected, "xmin": x, "xmax": x + w, "ymin": y, "ymax": y + h}
        
        return {"status": detected, "xmin": None, "xmax": None, "ymin": None, "ymax": None}
    
    def get_midpoints(self, frame):
        """
        Detect both red and blue rectangles in the frame and return their midpoints.
        """
        blue_info = self.detect_blue(frame)
        red_info = self.detect_red(frame)
        midpoints = {}

        if blue_info["status"]:
            blue_xmin = blue_info["xmin"]
            blue_xmax = blue_info["xmax"]
            blue_ymin = blue_info["ymin"]
            blue_ymax = blue_info["ymax"]
            cv2.rectangle(frame, (blue_xmin, blue_ymin), (blue_xmax, blue_ymax), (255, 0, 0), 2)  # Draw bounding box
            blue_midpoint = self.get_bbox_center(blue_info)
            midpoints["blue"] = blue_midpoint
            cv2.putText(frame, f"Blue ({blue_midpoint[0]}, {blue_midpoint[1]})", blue_midpoint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)  # Annotate midpoint

        if red_info["status"]:
            red_xmin = red_info["xmin"]
            red_xmax = red_info["xmax"]
            red_ymin = red_info["ymin"]
            red_ymax = red_info["ymax"]
            cv2.rectangle(frame, (red_xmin, red_ymin), (red_xmax, red_ymax), (0, 0, 255), 2)  # Draw bounding box
            red_midpoint = self.get_bbox_center(red_info)
            midpoints["red"] = red_midpoint
            cv2.putText(frame, f"Red ({red_midpoint[0]}, {red_midpoint[1]})", red_midpoint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)  # Annotate midpoint

        return midpoints
    
    def calculate_movement(self, midpoints, frame_shape):
        """
        Calculate the forward, lateral, depth, and yaw movements based on midpoints.
        """
        height, width = frame_shape
        center_x = width // 2
        center_y = height // 2

        forward = lateral = depth = yaw = 0

        target_color = self.config  # Use the config to determine which color to track
        if target_color in midpoints:
            target_midpoint = midpoints[target_color]
        else:
            return forward, lateral, depth, yaw

        # Calculate forward and depth based on vertical position
        forward = (center_y - target_midpoint[1]) / center_y
        depth = (target_midpoint[1] - center_y) / center_y
        
        # Calculate lateral based on horizontal position
        lateral = (target_midpoint[0] - center_x) / center_x
        
        # Calculate yaw based on horizontal position
        yaw = (target_midpoint[0] - center_x) / center_x

        # We want to yaw, then move laterally, then move forward.
        if abs(yaw) > self.threshold:
            forward = 0
            lateral = 0

        if abs(yaw) < self.threshold and abs(lateral) > self.threshold:
            yaw = 0
            forward = 0

        if abs(forward) > self.threshold and abs(lateral) < self.threshold and abs(yaw) < self.threshold:
            yaw = 0
            lateral = 0

        return forward, lateral, depth, yaw
    
    def run(self, raw_frame):
        """
        Process the input frame and calculate movements.
        """
        midpoints = self.get_midpoints(raw_frame)
        frame_shape = raw_frame.shape[:2]
        forward, lateral, depth, yaw = self.calculate_movement(midpoints, frame_shape)
        self.drop(forward, lateral, depth, yaw)

        # Annotate movements on the frame
        cv2.putText(raw_frame, f"Forward: {forward:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(raw_frame, f"Lateral: {lateral:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(raw_frame, f"Depth: {depth:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(raw_frame, f"Yaw: {yaw:.2f}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)        
        
        if self.drop_ball == True:
            self.end = True
        return {"lateral" : lateral, "forward" : forward, "yaw" : yaw, "drop_ball" : self.drop_ball, "end" : self.end}, raw_frame
    
    def drop(self, forward, lateral, depth, yaw):
        """
        Drop the object when the robot is aligned with the target midpoint.
        """
        if abs(forward) < 0.1 and abs(lateral) < 0.1 and abs(depth) < 0.1 and abs(yaw) < 0.1:
            print("[INFO] Dropping object at target location")
            self.drop_ball = True
            # Code to trigger the drop mechanism of the robot
        else:
            print("[INFO] Aligning with target, cannot drop object yet")

if __name__ == "__main__":
    # video_root_path = "/Users/brandontran3/downloads/Training Data/"
    video_root_path = "/home/kc/Desktop/Team Inspiration/RoboSub 2024/Training Data/"
    mission_name = "Bins/"
    video_name = "Bins Video 5.mp4"
    video_path = os.path.join(video_root_path, mission_name, video_name)

    print(f"Video path: {video_path}")

    # Change "red" or "blue" to test different target colors
    target_color = "red"  # Specify "red" or "blue" as the target color
    cv = CV(target_color)

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

                # Break the loop when 'q' key is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                print(f"Motion values: {motion_values}")

            cap.release()
            cv2.destroyAllWindows()
