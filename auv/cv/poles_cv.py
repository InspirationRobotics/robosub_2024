import cv2
import numpy as np
import time
import os

class CV:
    #camera = "/Users/avikaprasad/Downloads/poles_test_2.mp4"  # Input video
    camera = "/auv/camera/videoUSBRaw0"  # Camera stream for AUV

    def __init__(self, side="left", **config):
        self.shape = (640, 480)  # Frame size
        self.x_midpoint = self.shape[0] / 2
        self.y_midpoint = self.shape[1] / 2
        self.tolerance = 50
        self.side = side  # "left" or "right"
        self.last_seen_side = None

        print(f"[INFO] Pole CV initialized for {self.side}-side navigation")

    def detect_red_white_centers(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red color mask (two ranges in HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # White color mask
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])

        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        def largest_center(contours):
            if not contours:
                return None
            cnt = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(cnt)
            return x + w / 2

        red_center = largest_center(red_contours)
        white_center = largest_center(white_contours)

        if red_center and white_center:
            center = (red_center + white_center) / 2
        else:
            center = None

        return center, red_mask | white_mask

    def movement_calculation(self, center):
        forward = 1.5
        lateral = 0
        yaw = 0

        if center is not None:
            frame_width = self.shape[0]
            left_thresh = frame_width * 0.2
            right_thresh = frame_width * 0.8

            if center < left_thresh:
                yaw = 0.4
                print("[INFO] Center left of frame → veer right")
            elif center > right_thresh:
                yaw = -0.4
                print("[INFO] Center right of frame → veer left")
            else:
                yaw = 0
                print("[INFO] Centered → go straight")

            self.last_seen_side = "left" if center < self.x_midpoint else "right"
        else:
            forward = 0
            if self.last_seen_side == "left":
                yaw = 0.5  # Clockwise
                print("[INFO] No poles → last seen left → spin CW")
            elif self.last_seen_side == "right":
                yaw = -0.5  # Counter-clockwise
                print("[INFO] No poles → last seen right → spin CCW")
            else:
                yaw = 0
                print("[INFO] No poles detected → holding position")

        print(f"[MOVEMENT] Forward: {forward}, Lateral: {lateral}, Yaw: {yaw}")
        return forward, lateral, yaw, 0

    def run(self, raw_frame, target, detections):
        center, mask = self.detect_red_white_centers(raw_frame)
        visualized_frame = cv2.bitwise_and(raw_frame, raw_frame, mask=mask)
        forward, lateral, yaw, vertical = self.movement_calculation(center)

        return {
            "forward": forward,
            "lateral": lateral,
            "yaw": yaw,
            "vertical": vertical,
            "end": False
        }, visualized_frame

if __name__ == "__main__":
    cv = CV(side="left")
    cap = cv2.VideoCapture(cv.camera)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        center, mask = cv.detect_red_white_centers(frame)
        movement = cv.movement_calculation(center)
        
        cv2.imshow("Red & White Mask", mask)
        print(f"[FRAME] Movement: {movement}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()