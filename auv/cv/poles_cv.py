import cv2
import numpy as np
import time
import os

class CV:
    camera = "/Users/avikaprasad/Downloads/poles_test_2.mp4" 

    def __init__(self, **config):
        self.shape = (640, 480)
        self.x_midpoint = self.shape[0] / 2
        self.y_midpoint = self.shape[1] / 2
        self.tolerance = 50
        self.slalom_direction = 1  # 1 = right, -1 = left
        self.prev_pole_x = None
        self.last_depth_change = time.time()

        print("[INFO] Slalom CV initialized")

    def detect_red_poles(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red has two hue ranges in HSV
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 | mask2

        # Apply morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        red_poles = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 300:  # Filter out small blobs
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = h / float(w)
                if aspect_ratio > 2.5:  # Likely a vertical pole
                    red_poles.append((x, y, w, h))

        if red_poles:
            print("[DEBUG] ✅ Successfully detected red pole(s)!")

        return red_poles, red_mask

    def movement_calculation(self, poles):
        forward = 1.5
        lateral = 0
        yaw = 0
        vertical = 0

        if poles:
            # Choose the nearest (largest) red pole
            target_pole = max(poles, key=lambda box: box[2] * box[3])
            x, y, w, h = target_pole
            pole_center_x = x + w / 2

            # Slalom: alternate direction each time we pass a pole
            if self.prev_pole_x is not None and abs(pole_center_x - self.prev_pole_x) > 100:
                self.slalom_direction *= -1  # Flip slalom direction
                self.prev_pole_x = pole_center_x
            elif self.prev_pole_x is None:
                self.prev_pole_x = pole_center_x

            # Yaw to keep red pole at offset (left/right) of center
            offset = 100 * self.slalom_direction
            target_x = self.x_midpoint + offset

            if pole_center_x < target_x - self.tolerance:
                yaw = -0.5
            elif pole_center_x > target_x + self.tolerance:
                yaw = 0.5
            else:
                yaw = 0

            # Adjust vertical position to stay low
            if (y + h) < self.y_midpoint - 20 and time.time() - self.last_depth_change > 10:
                vertical = -0.05
                self.last_depth_change = time.time()

        else:
            # No red poles seen, spin to search
            yaw = 0.6
            self.prev_pole_x = None

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
            "end": False  # You could end after N slalom passes if desired
        }, visualized_frame

if __name__ == "__main__":
    import cv2

    cv = CV()
    cap = cv2.VideoCapture(cv.camera)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        poles, mask = cv.detect_red_poles(frame)
        movement = cv.movement_calculation(poles)

        # Draw detections on the frame
        for (x, y, w, h) in poles:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

        cv2.imshow("Detected Poles", frame)
        cv2.imshow("Red Mask", mask)

        print(f"Movement Command: {movement}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()