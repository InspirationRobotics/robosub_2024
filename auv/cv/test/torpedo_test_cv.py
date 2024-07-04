"""
Torpedo CV and logic test.

Author: Brandon Tran
"""

import time
import cv2
import numpy as np
import os

class CV:
    """
    CV class for Torpedo mission. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """

    def __init__(self, ref_img_path="samples/board.png"):
        """
        Initialize the CV class.
        """
        self.shape = (640, 480)  # Set the frame shape
        self.aligned = False
        self.detected = False
        self.step = 0
        self.end = False
        self.ref_img_path = ref_img_path
        self.reference_image = cv2.imread(self.ref_img_path)
        
        if self.reference_image is None:
            raise FileNotFoundError(f"Reference image not found at {ref_img_path}")
        
        self.reference_image = cv2.cvtColor(self.reference_image, cv2.COLOR_BGR2GRAY)
        self.reference_image = self.apply_clahe(self.reference_image)
        self.sift = cv2.SIFT_create()
        self.kp_ref, self.des_ref = self.sift.detectAndCompute(self.reference_image, None)
        self.bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
        
        print(f"[INFO] Torpedo CV Init")
    
    def apply_clahe(self, image):
        """
        Apply CLAHE to the image to improve contrast.
        """
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        return clahe.apply(image)

    def process_sift(self, frame):
        """
        Process the frame using SIFT feature matching.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = self.apply_clahe(gray)
        
        kp_frame, des_frame = self.sift.detectAndCompute(gray, None)
        matches = self.bf.match(self.des_ref, des_frame)
        matches = sorted(matches, key=lambda x: x.distance)
        
        matched_image = cv2.drawMatches(self.reference_image, self.kp_ref, gray, kp_frame, matches[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        return matched_image, matches

    def get_center(self, bbox):
        """
        Get the center of the bounding box.
        """
        x, y, w, h = bbox
        cx = x + w // 2
        cy = y + h // 2
        return (cx, cy)

    def find_torpedo_targets(self, frame):
        """
        Detect the torpedo targets in the frame.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detected_targets = []
        
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            if len(approx) == 8:  # Check for octagonal shapes
                area = cv2.contourArea(cnt)
                x, y, w, h = cv2.boundingRect(cnt)
                detected_targets.append((area, (x, y, w, h), cnt))
        
        detected_targets.sort(key=lambda x: x[0])
        return detected_targets

    def label_torpedo_sizes(self, frame, targets):
        """
        Label the sizes of the detected torpedo targets.
        """
        size_labels = ["smallest", "small", "large", "largest"]
        labeled_targets = []
        
        for i, target in enumerate(targets[:4]):
            area, bbox, cnt = target
            x, y, w, h = bbox
            label = size_labels[i]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            labeled_targets.append((label, self.get_center(bbox)))
        
        return labeled_targets

    def label_midpoints(self, frame, labeled_targets):
        """
        Label the midpoints of the detected torpedo targets.
        """
        for label, (cx, cy) in labeled_targets:
            cv2.putText(frame, f"{label} ({cx}, {cy})", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    
    def align_to_target(self, target):
        """
        Align the vehicle to the target.
        """
        label, (cx, cy) = target
        # Placeholder for alignment logic
        print(f"[INFO] Aligning to {label} target at ({cx}, {cy})")
    
    def fire_torpedo(self, target):
        """
        Fire the torpedo at the target.
        """
        label, (cx, cy) = target
        # Placeholder for firing logic
        print(f"[INFO] Firing torpedo at {label} target at ({cx}, {cy})")

    def run(self, raw_frame):
        """
        Process the input frame and perform torpedo detection and firing logic.
        """
        raw_frame = cv2.resize(raw_frame, self.shape)
        
        detected_targets = self.find_torpedo_targets(raw_frame)
        if len(detected_targets) < 2:
            print("[ERROR] Could not find both small and next smallest torpedo targets.")
            return raw_frame
        
        labeled_targets = self.label_torpedo_sizes(raw_frame, detected_targets)
        self.label_midpoints(raw_frame, labeled_targets)
        
        if len(labeled_targets) >= 2:
            self.align_to_target(labeled_targets[0])
            self.fire_torpedo(labeled_targets[0])
            
            time.sleep(2)
            
            self.align_to_target(labeled_targets[1])
            self.fire_torpedo(labeled_targets[1])

        return raw_frame

if __name__ == "__main__":
    # video_root_path = "/Users/brandontran3/downloads/Training Data/"
    video_root_path = "/Users/brandontran3/downloads/Training Data/"
    mission_name = "Torpedo/"
    video_name = "Torpedo Video 5.mp4"
    video_path = os.path.join(video_root_path, mission_name, video_name)

    print(f"Video path: {video_path}")

    cv = CV()

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

                viz_frame = cv.run(frame)
                if viz_frame is not None:
                    cv2.imshow("frame", viz_frame)

                # Break the loop when 'q' key is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            cap.release()
            cv2.destroyAllWindows()
