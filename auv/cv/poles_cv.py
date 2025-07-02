"""
Pole Slalom CV. Detects the red pole, approaches it, aligns based on prior gate direction,
and repeats the maneuver three times to complete the mission.
"""

import cv2
import time
import numpy as np
import os

class CV:
    camera = "/auv/camera/videoOAKdRawForward"

    def __init__(self, **config):
        self.shape = (640, 480)
        self.x_midpoint = self.shape[0] / 2
        self.tolerance = 50  # Pixel offset for alignment
        self.row_counter = 0
        self.max_rows = 3
        self.config = config
        self.side = config.get("side", "right")  # "left" or "right", passed from gate mission
        self.state = "searching"
        self.red_area_threshold = config.get("red_area_threshold", 20000)
        self.end = False
        self.lost_target = False
        self.depth_time = time.time()
        print("[INFO] Pole Slalom CV initialized")

    def detect_red_pole(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        red_poles = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:  # Noise filter
                x, y, w, h = cv2.boundingRect(cnt)
                red_poles.append((x, y, w, h, area))

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
        return {"status": False, "xmin": None, "xmax": None, "ymin": None, "ymax": None, "area": 0}, red_mask

    def movement_calculation(self, detection):
        forward = 0
        lateral = 0
        yaw = 0
        vertical = 0

        if self.row_counter >= self.max_rows:
            self.end = True
            return forward, lateral, yaw, vertical

        if detection["status"]:
            x_center = (detection["xmin"] + detection["xmax"]) / 2
            area = detection["area"]

            if self.state == "searching":
                yaw = -0.7 if self.side == "right" else 0.7
                if area > 1000:
                    self.state = "approaching"

            elif self.state == "approaching":
                yaw = 0
                if area < self.red_area_threshold:
                    forward = 1.5
                else:
                    forward = 0
                    self.state = "aligning"

            elif self.state == "aligning":
                if self.side == "right":
                    lateral = 0.8
                    if x_center > self.shape[0]:
                        self.state = "moving_forward"
                else:
                    lateral = -0.8
                    if x_center < 0:
                        self.state = "moving_forward"

            elif self.state == "moving_forward":
                forward = 1.0
                yaw = -0.5 if self.side == "right" else 0.5
                self.row_counter += 1
                self.state = "searching"

        else:
            if self.state in ["searching", "approaching"]:
                yaw = -0.7 if self.side == "right" else 0.7
            elif self.state == "aligning":
                lateral = 0.8 if self.side == "right" else -0.8
            elif self.state == "moving_forward":
                forward = 1.0
                yaw = -0.5 if self.side == "right" else 0.5

        return forward, lateral, yaw, vertical

    def run(self, raw_frame, target, detections):
        visualized_frame = None
        detection, mask = self.detect_red_pole(raw_frame)

        if raw_frame is not None:
            frame = raw_frame.copy()

            if detection["status"]:
                # Draw bounding box and center lines
                x1, x2 = detection["xmin"], detection["xmax"]
                y1, y2 = detection["ymin"], detection["ymax"]
                x_center = int((x1 + x2) / 2)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.line(frame, (x_center, 0), (x_center, self.shape[1]), (255, 255, 0), 2)
                cv2.line(frame, (int(self.x_midpoint), 0), (int(self.x_midpoint), self.shape[1]), (0, 255, 0), 1)

            # State text
            cv2.putText(frame, f"State: {self.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 255, 200), 2)
            cv2.putText(frame, f"Row: {self.row_counter}/{self.max_rows}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 255, 200), 2)

            visualized_frame = frame
            cv2.imshow("Pole Detection", frame)
            cv2.imshow("Red Mask", mask)

        forward, lateral, yaw, vertical = self.movement_calculation(detection)

        return {"lateral": lateral, "forward": forward, "yaw": yaw, "vertical": vertical, "end": self.end}, visualized_frame