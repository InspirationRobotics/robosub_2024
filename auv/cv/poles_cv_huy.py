import cv2
import numpy as np
import time

class CV:
    camera = "/auv/camera/videoOAKdRawForward" # Local video file

    def __init__(self, side="left", **config):
        self.shape = (640, 480)
        self.x_midpoint = self.shape[0] / 2
        self.y_midpoint = self.shape[1] / 2
        self.tolerance = 50
        self.side = side  # side="left" or side="right" as decided from the gate mission!
        self.last_seen_side = None
        print(f"[INFO] Dual Pole CV with offset logic for {self.side}-side navigation")

    def enhance_frame(self, frame):
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        l = clahe.apply(l)
        enhanced = cv2.merge((l, a, b))
        return cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)

    def detect_poles(self, frame):
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        frame = self.enhance_frame(frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red mask
        lower_red1 = np.array([0, 80, 50])
        upper_red1 = np.array([12, 255, 255])
        lower_red2 = np.array([168, 80, 50])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

        # White mask
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 60, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)

        # Find contours for each color
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_white, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        red_poles = []
        white_poles = []

        for cnt in contours_red:
            area = cv2.contourArea(cnt)
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = h / float(w + 1e-5)
            if area > 200 and aspect_ratio > 2.5 and w < 100:
                red_poles.append((x, y, w, h))
        for cnt in contours_white:
            area = cv2.contourArea(cnt)
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = h / float(w + 1e-5)
            if area > 200 and aspect_ratio > 2.5 and w < 100:
                white_poles.append((x, y, w, h))

        return red_poles, white_poles, mask_red, mask_white, frame

    def draw_poles(self, frame, red_poles, white_poles, red_center=None, offset=None):
        for (x, y, w, h) in red_poles:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0,0,255), 2)
        for (x, y, w, h) in white_poles:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
        # === DEBUG LINES ===
        if red_center is not None: 
            cv2.line(frame, (int(red_center), 0), (int(red_center), frame.shape[0]), (0,0,255), 1)   # Red center
            if offset is not None:
                offset_x = int(red_center + offset) if self.side == "left" else int(red_center - offset)
                cv2.line(frame, (offset_x, 0), (offset_x, frame.shape[0]), (255,0,0), 2)             # Blue offset
        return frame

    def get_main_pole_centers(self, red_poles, white_poles):
        red_center = None
        white_center = None
        if red_poles:
            largest_red = max(red_poles, key=lambda b: b[2]*b[3])
            x, y, w, h = largest_red
            red_center = x + w/2
        if white_poles:
            largest_white = max(white_poles, key=lambda b: b[2]*b[3])
            x, y, w, h = largest_white
            white_center = x + w/2
        return red_center, white_center

    def movement_calculation(self, red_center, white_center):
        forward = 1.5
        lateral = 0
        yaw = 0

        desired_offset = 120  # *** TUNE THIS VALUE ***

        if red_center is not None and white_center is not None:
            center = (red_center + white_center) / 2
            frame_width = self.shape[0]
            left_thresh = frame_width * 0.25
            right_thresh = frame_width * 0.75

            if center < left_thresh:
                yaw = 1.0
                print("[INFO] Midpoint left → veer right")
            elif center > right_thresh:
                yaw = -1.0
                print("[INFO] Midpoint right → veer left")
            else:
                yaw = 0
                print("[INFO] Centered between poles → go straight")
            self.last_seen_side = "left" if center < self.x_midpoint else "right"

        elif red_center is not None:
            # Only red: offset depends on which side we're supposed to pass
            if self.side == "left":
                target_pos = red_center + desired_offset   # Keep red on left, sub on right
            else:
                target_pos = red_center - desired_offset   # Keep red on right, sub on left

            if target_pos < self.x_midpoint - self.tolerance:
                yaw = 1.0
                print(f"[INFO] Red+offset left → veer right (side: {self.side})")
            elif target_pos > self.x_midpoint + self.tolerance:
                yaw = -1.0
                print(f"[INFO] Red+offset right → veer left (side: {self.side})")
            else:
                yaw = 0
                print(f"[INFO] Following red pole with offset → go straight (side: {self.side})")

            if target_pos < self.x_midpoint:
                self.last_seen_side = "left"
            else:
                self.last_seen_side = "right"

        else:
            forward = 0
            if self.last_seen_side == "left":
                yaw = 1
                print("[INFO] Lost poles, last left → spin CW")
            elif self.last_seen_side == "right":
                yaw = -1
                print("[INFO] Lost poles, last right → spin CCW")
            else:
                yaw = 0
                print("[INFO] No poles detected → holding position")
        print(f"[MOVEMENT] Forward: {forward}, Lateral: {lateral}, Yaw: {yaw}")
        return forward, lateral, yaw, 0

    def run(self, raw_frame, target, detections):
        red_poles, white_poles, mask_red, mask_white, enhanced = self.detect_poles(raw_frame)
        red_center, white_center = self.get_main_pole_centers(red_poles, white_poles)
        offset_for_line = 120 if red_center is not None and white_center is None else None
        frame_drawn = self.draw_poles(
            enhanced.copy(),
            red_poles,
            white_poles,
            red_center=red_center,
            offset=offset_for_line
        )
        combined_mask = cv2.bitwise_or(mask_red, mask_white)
        visualized_frame = cv2.bitwise_and(frame_drawn, frame_drawn, mask=combined_mask)
        forward, lateral, yaw, vertical = self.movement_calculation(red_center, white_center)
        return {
            "forward": forward,
            "lateral": lateral,
            "yaw": yaw,
            "vertical": vertical,
            "end": False
        }, visualized_frame

if __name__ == "__main__":
    # Use side="left" or "right" based on the GATE decision
    cv = CV(side="left")  # Change to "right" if gate chooses that path
    cap = cv2.VideoCapture(cv.camera)
    desired_offset = 120  # <-- TUNE THIS as needed!

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        red_poles, white_poles, mask_red, mask_white, enhanced = cv.detect_poles(frame)
        red_center, white_center = cv.get_main_pole_centers(red_poles, white_poles)

        offset_for_line = desired_offset if red_center is not None and white_center is None else None
        frame_drawn = cv.draw_poles(
            enhanced.copy(),
            red_poles,
            white_poles,
            red_center=red_center,
            offset=offset_for_line
        )

        cv2.imshow("Red Poles Mask", mask_red)
        cv2.imshow("White Poles Mask", mask_white)
        cv2.imshow("Poles Detection", frame_drawn)

        movement = cv.movement_calculation(red_center, white_center)
        print(f"[FRAME] Movement: {movement}")

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
