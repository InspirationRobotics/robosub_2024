import time
import cv2
import numpy as np

class CV:
    camera = "/auv/camera/videoOAKdRawForward"

    def __init__(self, **config):
        self.strafe_direction = config.get("strafe_direction", "right")
        self.red_size_threshold = config.get("red_size_threshold", 10000)
        self.forward_duration = config.get("forward_duration", 30)
        self.state = "approach"
        self.last_red_seen = None
        self.forward_counter = 0
        self.yaw_direction = -1 if self.strafe_direction == "right" else 1
        self.completed_rows = 0
        print("[INFO] Slalom CV initialized")

    def detect_red_poles(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_boxes = []

        for c in contours:
            area = cv2.contourArea(c)
            if area > 300:
                x, y, w, h = cv2.boundingRect(c)
                red_boxes.append((x, y, w, h, area))

        return sorted(red_boxes, key=lambda b: b[4], reverse=True)

    def run(self, frame, target=None, detections=None):
        motion = {"lateral": 0, "forward": 0, "end": False}
        red_boxes = self.detect_red_poles(frame)

        if self.completed_rows >= 3:
            motion["end"] = True
            print("[INFO] All red poles passed. Mission complete.")
            return motion, frame

        if self.state == "approach":
            if red_boxes:
                target_box = red_boxes[0]
                x, y, w, h, area = target_box
                self.last_red_seen = target_box
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                motion["forward"] = 1

                if area > self.red_size_threshold:
                    print(f"[INFO] Reached red pole with area: {area}. Starting strafe.")
                    self.state = "strafe"
            else:
                print("[WARN] Red pole lost. Slowing or stopping.")
                motion["forward"] = 0

        elif self.state == "strafe":
            motion["lateral"] = 1 if self.strafe_direction == "right" else -1
            if not red_boxes:
                print("[INFO] Red pole no longer visible. Starting forward movement.")
                self.state = "post_strafe_forward"
                self.forward_counter = 0

        elif self.state == "post_strafe_forward":
            motion["forward"] = 1
            self.forward_counter += 1
            if self.forward_counter > self.forward_duration:
                print("[INFO] Finished forward motion. Starting yaw to reacquire red pole.")
                self.state = "yaw"
                self.forward_counter = 0

        elif self.state == "yaw":
            motion["lateral"] = 0
            motion["forward"] = 0
            frame = self.add_yaw_marker(frame)
            if red_boxes:
                print("[INFO] Found next red pole. Resuming approach.")
                self.completed_rows += 1
                self.state = "approach"
            else:
                motion["lateral"] = 0
                motion["yaw"] = self.yaw_direction

        return motion, frame

    def add_yaw_marker(self, frame):
        h, w, _ = frame.shape
        center_x = w // 2
        cv2.line(frame, (center_x, 0), (center_x, h), (255, 255, 0), 2)
        return frame


# -------------- TESTER CODE ------------------

if __name__ == "__main__":
    tester_video_path = "/Users/avikaprasad/Downloads/poles_test_2.mp4"  # Update path as needed
    cap = cv2.VideoCapture(tester_video_path)

    cv_instance = CV(strafe_direction="right")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("[INFO] End of video or failed to load frame.")
            break

        motion, output_frame = cv_instance.run(frame, target=None, detections=None)
        print("[MOTION OUTPUT]:", motion)

        cv2.imshow("Slalom Detection", output_frame)

        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()