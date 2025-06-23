import cv2
import numpy as np
import time
import os

class CV:
    camera = "/Users/avikaprasad/Downloads/poles_test_2.mp4"  # Change as needed

    def __init__(self, target="Left", **config):
        self.shape = (640, 480)
        self.x_midpoint = self.shape[0] / 2
        self.y_midpoint = self.shape[1] / 2
        self.tolerance = 50
        self.target_side = target  # "Left" or "Right" based on gate mission
        self.last_seen_side = None  # "Left" or "Right"
        self.last_depth_change = time.time()
        print("[INFO] Slalom CV initialized with target side:", self.target_side)

    def detect_red_poles(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 | mask2

        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        red_poles = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 300:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = h / float(w)
                if aspect_ratio > 2.5:
                    red_poles.append((x, y, w, h))

        if red_poles:
            print("[DEBUG] ✅ Successfully detected red pole(s)!")

        return red_poles, red_mask

    def movement_calculation(self, poles):
        forward = 0
        lateral = 0
        yaw = 0
        vertical = 0

        if poles:
            # Select largest red pole
            target_pole = max(poles, key=lambda box: box[2] * box[3])
            x, y, w, h = target_pole
            pole_center_x = x + w / 2

            # Track last seen side
            if pole_center_x < self.x_midpoint:
                self.last_seen_side = "Left"
            else:
                self.last_seen_side = "Right"

            # Step 1: Move forward until pole fits nicely in frame (about 20% of width)
            area = w * h
            if area < 10000:
                forward = 1.5
                return forward, lateral, yaw, vertical

            # Step 2: Slide to assigned side (Left or Right)
            if self.target_side == "Left":
                lateral = -1.0
            else:
                lateral = 1.0

            # Step 3: Adjust yaw depending on pole x-position
            if pole_center_x < self.shape[0] * 0.2:
                yaw = -0.5  # Veer left
                forward = 1.0
            elif pole_center_x > self.shape[0] * 0.8:
                yaw = 0.5   # Veer right
                forward = 1.0
            else:
                yaw = 0     # Go straight
                forward = 1.5

            # Bonus: Stay low (below midline)
            if (y + h) < self.y_midpoint - 20 and time.time() - self.last_depth_change > 10:
                vertical = -0.05
                self.last_depth_change = time.time()

        else:
            # No pole detected: yaw in place based on last seen
            if self.last_seen_side == "Left":
                yaw = 0.5  # spin clockwise
            elif self.last_seen_side == "Right":
                yaw = -0.5  # spin counter-clockwise
            else:
                yaw = 0.6  # default spin

        return forward, lateral, yaw, vertical

    def run(self, raw_frame, target, detections):
        poles, mask = self.detect_red_poles(raw_frame)
        visualized_frame = cv2.bitwise_and(raw_frame, raw_frame, mask=mask)

        forward, lateral, yaw, vertical = self.movement_calculation(poles)

        return {
            "forward": forward,
            "lateral": lateral,
            "yaw": yaw,
            "vertical": vertical,
            "end": False
        }, visualized_frame

if __name__ == "__main__":
    cv = CV(target="Right")  # Set to "Left" or "Right" based on animal choice
    cap = cv2.VideoCapture(cv.camera)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        poles, mask = cv.detect_red_poles(frame)
        movement = cv.movement_calculation(poles)

        for (x, y, w, h) in poles:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

        cv2.imshow("Detected Poles", frame)
        cv2.imshow("Red Mask", mask)

        print(f"[MOVEMENT] F:{movement[0]}, L:{movement[1]}, Y:{movement[2]}, V:{movement[3]}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()