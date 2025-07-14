"""
Pole Slalom CV. Detects red poles, yaws to face it, and approaches until close.
"""

import cv2
import time
import numpy as np
import os

import cv2
import numpy as np
import time

class CV:
    camera = "/auv/camera/videoOAKdRawForward"

    def __init__(self, **config):
        self.shape = None  # Will set this dynamically
        self.x_midpoint = None
        self.tolerance = 40  # How centered the object should be
        self.config = config
        self.state = "searching"  # searching → centering → approaching
        self.end = False
        self.start_time = time.time()

        print("[INFO] Pole Center & Approach CV initialized")

    def detect_red_pole(self, frame):
        # Set shape dynamically on first frame
        if self.shape is None:
            height, width = frame.shape[:2]
            self.shape = (width, height)
            self.x_midpoint = width / 2
            print(f"[INFO] Frame shape set dynamically: width={width}, height={height}")

        # Crop a bit off the bottom (e.g., last 100 pixels)
        crop_bottom = 100
        cropped_frame = frame[0:self.shape[1] - crop_bottom, 0:self.shape[0]]

        # Convert to HSV and create red mask
        hsv = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([20, 0, 0])
        upper_red1 = np.array([255, 255, 50])
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1)

        # Find contours
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        red_poles = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:
                x, y, w, h = cv2.boundingRect(cnt)
                red_poles.append((x, y, w, h, area))

        # If any red pole found, return the largest one
        if red_poles:
            red_poles.sort(key=lambda x: x[4], reverse=True)
            x, y, w, h, area = red_poles[0]
            return {
                "status": True,
                "xmin": x,
                "xmax": x + w,
                "ymin": y,
                "ymax": y + h,
                "area": area
            }, red_mask

        return {"status": False, "xmin": None, "xmax": None,"ymin": None,"ymax": None,"area": 0}, red_mask

    def movement_calculation(self, detection):
        forward = 0
        lateral = 0
        yaw = 0
        vertical = 0

        if self.state == "searching":
            if detection["status"]:
                self.state = "centering"
            else:
                # Spin in place to search
                yaw = 1.5
                print("[INFO] Searching: No red pole detected → yawing")

        elif self.state == "centering":
            if detection["status"]:
                x_center = (detection["xmin"] + detection["xmax"]) / 2
                offset = x_center - self.x_midpoint

                if abs(offset) > self.tolerance:
                    yaw = -1.0 if offset > 0 else 1.0  # Yaw toward the center
                    print(f"[INFO] Centering: offset={offset:.1f} → yaw={yaw}")
                else:
                    print("[INFO] Centering: Pole centered → transitioning to approaching")
                    self.state = "approaching"
            else:
                print("[WARN] Lost pole while centering → reverting to searching")
                self.state = "searching"

        elif self.state == "approaching":
            if detection["status"]:
                area = detection["area"]
                if area < 16000:
                    forward = 2.0
                    print(f"[INFO] Approaching: area={area:.0f} → moving forward")
                else:
                    forward = 0
                    self.end = True
                    print("[INFO] Pole is close enough → stopping")
                    self.state = "strafing"
            else:
                print("[WARN] Lost pole while approaching → reverting to searching")
                self.state = "searching"
        
        elif self.state == "strafing":
            while time.time() - self.start_time < 1.5:  # Strafing for 1.5 seconds
                lateral = 2.0
                print(f"[INFO] Strafing: Moving laterally to come in between poles")
                break

        return forward, lateral, yaw, vertical

    def run(self, raw_frame, target, detections):
        detection, mask = self.detect_red_pole(raw_frame)
        forward, lateral, yaw, vertical = self.movement_calculation(detection)

        # Visualization
        frame = raw_frame.copy()
        if detection["status"]:
            x1, y1 = detection["xmin"], detection["ymin"]
            x2, y2 = detection["xmax"], detection["ymax"]
            x_center = int((x1 + x2) / 2)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.line(frame, (x_center, 0), (x_center, self.shape[1]), (255, 255, 0), 2)
            cv2.line(frame, (int(self.x_midpoint), 0), (int(self.x_midpoint), self.shape[1]), (0, 255, 0), 1)
            cv2.putText(frame, f"Area: {detection['area']}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

        cv2.putText(frame, f"State: {self.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

        return {"lateral": lateral, "forward": forward, "yaw": yaw,"vertical": vertical, "end": self.end}, frame