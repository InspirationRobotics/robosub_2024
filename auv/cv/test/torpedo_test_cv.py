"""
Torpedo CV and logic test.

Author: Brandon Tran
"""

import argparse
import math
import os
import time
import cv2
import numpy as np

file_dir = os.path.dirname(os.path.abspath(__file__))

class CV:
    camera = 0  # Change this to your camera source, e.g., "/auv/camera/videoUSBRaw0" if needed

    def __init__(self, **config):
        """
        Init of torpedo CV,
        """
        self.reference_images = {
            "torpedo_board": cv2.imread(f"{file_dir}/reference/Torpedo Board.png"),
            "torpedo_smallest": cv2.imread(f"{file_dir}/reference/Torpedo Smallest.png"),
            "torpedo_large": cv2.imread(f"{file_dir}/reference/Torpedo Large.png"),
            "torpedo_largest": cv2.imread(f"{file_dir}/reference/Torpedo Largest.png"),
            "torpedo_small": cv2.imread(f"{file_dir}/reference/Torpedo Small.png"),
        }

        for name, img in self.reference_images.items():
            assert img is not None, f"Image for {name} not found."

        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.sift = cv2.SIFT_create()
        self.bf = cv2.BFMatcher()

        self.keypoints_descriptors = {
            name: self.sift.detectAndCompute(img, None)
            for name, img in self.reference_images.items()
        }

        self.ref_shapes = {
            name: img.shape for name, img in self.reference_images.items()
        }

        self.shape = (480, 640, 3)
        self.step = 0

        self.center_positions = {name: [] for name in self.reference_images.keys()}

        self.offset_center = 0.5
        self.target_coords = (0.0, 0.0)
        self.yaw_threshold = 4
        self.x_threshold = 0.1
        self.y_threshold = 0.1

        self.counter = 0
        self.aligned = True
        self.firing_range = 950
        self.near_range = 750
        self.fired1 = False
        self.fired2 = False

        print("[INFO] Torpedo cv Init")

    def equalize_clahe(self, image):
        """Equalize the histogram of the image"""
        b, g, r = cv2.split(image)
        b = self.clahe.apply(b)
        g = self.clahe.apply(g)
        r = self.clahe.apply(r)
        return cv2.merge((b, g, r))

    def equilize(img):
        """Equilize the histogram of the image"""
        b, g, r = cv2.split(img)
        b = cv2.equalizeHist(b)
        g = cv2.equalizeHist(g)
        r = cv2.equalizeHist(r)
        return cv2.merge((b, g, r))

    def process_sift(self, ref_img, img, kp1, des1, kp2=None, des2=None, threshold=0.65, window_viz=None):
        """Sift matching with reference points if kp2 and des2 are not given, it will compute them"""
        if kp2 is None or des2 is None:
            kp2, des2 = self.sift.detectAndCompute(img, None)
        matches = self.bf.knnMatch(des1, des2, k=2)
        good = []
        for m, n in matches:
            if m.distance < threshold * n.distance:
                good.append(m)

        if len(good) < 4:
            return None

        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        if window_viz is not None:
            img = cv2.drawMatches(ref_img, kp1, img, kp2, good, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            cv2.imshow(window_viz, img)
        return H

    def get_center(self, H, src_shape, norm=False):
        """Get center of a given homography H"""
        h, w, _ = src_shape
        center = np.array([[w / 2, h / 2]]).reshape(-1, 1, 2)
        center = cv2.perspectiveTransform(center, H).astype(np.int32)[0][0]
        if norm:
            center = [(center[0] - 320) / 320, (center[1] - 240) / 240]
        return center

    def projection(self, H, src_shape, points_normalized):
        """Get projection of a given homography H Points are normalized between -1 and 1"""
        h, w, _ = src_shape
        points = np.array(points_normalized)
        points[:, 0] = points[:, 0] * w // 2 + w // 2
        points[:, 1] = points[:, 1] * h // 2 + h // 2
        points = points.reshape(-1, 1, 2).astype(np.float32)
        points = cv2.perspectiveTransform(points, H).astype(np.int32).reshape(-1, 2)
        return points

    def get_orientation(self, H, src_shape):
        h, w, _ = src_shape
        pts = (
            np.array(
                [
                    [0, 0],
                    [w, 0],
                    [w, h],
                    [0, h],
                ]
            )
            .reshape(-1, 1, 2)
            .astype(np.float32)
        )
        pts = cv2.perspectiveTransform(pts, H).astype(np.int32).reshape(4, 2)

        # get an estimation of the Y axis rotation
        left_h_dist = np.linalg.norm(pts[0] - pts[3])
        right_h_dist = np.linalg.norm(pts[1] - pts[2])
        width = w / np.linalg.norm(pts[0] - pts[1])

        yaw = math.atan((left_h_dist - right_h_dist) / (left_h_dist + right_h_dist))
        yaw = math.degrees(yaw)
        print(f"[INFO] Yaw: {yaw}")
        return yaw, width
    
    def detect_board(self, img):
        """Detect the board in the image."""
        ref_img = self.reference_images["torpedo_board"]
        kp1, des1 = self.keypoints_descriptors["torpedo_board"]
        H = self.process_sift(ref_img, img, kp1, des1, threshold=0.65)
        if H is not None:
            center = self.get_center(H, self.ref_shapes["torpedo_board"])
            bbox = self.get_bounding_box(H, self.ref_shapes["torpedo_board"])
            size = self.get_bbox_size(bbox)
            return {"name": "torpedo_board", "center": center, "bbox": bbox, "size": size}
        return None

    def detect_holes(self, img):
        """Detect all the holes in the image and their positions and return bounding boxes."""
        kp2, des2 = self.sift.detectAndCompute(img, None)
        detections = []
        for name, (kp1, des1) in self.keypoints_descriptors.items():
            H = self.process_sift(self.reference_images[name], img, kp1, des1, kp2, des2, threshold=0.65)
            if H is not None:
                center = self.get_center(H, self.ref_shapes[name])
                bbox = self.get_bounding_box(H, self.ref_shapes[name])
                size = self.get_bbox_size(bbox)
                detections.append({"name": name, "center": center, "bbox": bbox, "size": size})
        return detections

    def get_bounding_box(self, H, src_shape):
        """Get bounding box of a given homography H."""
        h, w, _ = src_shape
        pts = np.float32([[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, H)
        return np.int32(dst)

    def get_bbox_size(self, bbox):
        """Calculate the size of the bounding box."""
        x, y, w, h = cv2.boundingRect(bbox)
        return w * h

    def align_yaw(self, H, ref_shape):
        yaw, dist = self.get_orientation(H, ref_shape)
        yaw_required = (self.yaw_threshold < yaw < -self.yaw_threshold)
        if yaw_required:
            yaw = np.clip(yaw * 1, -1, 1)
        else:
            yaw = 0
            if self.aligned:
                self.step = 2
                self.aligned = False
            else:
                self.aligned = True
        return yaw_required, yaw, dist

    def align_lateral(self, target):
        lateral = np.clip(target[0] * 3.5, -1, 1)
        lateral_required = (self.x_threshold < lateral < -self.x_threshold)
        if not lateral_required:
            lateral = 0
            if self.aligned:
                self.step = 3
                self.aligned = False
            else:
                self.aligned = True
        return lateral_required, lateral

    def align_depth(self, target):
        vertical = np.clip(target[1] * 3.5, -1, 1)
        depth_required = (self.y_threshold < vertical < -self.y_threshold)
        if not depth_required:
            vertical = 0
            if self.aligned:
                self.step = 1
                self.aligned = False
            else:
                self.aligned = True
        return depth_required, vertical
    
    def update_center(self, img):
        detections = self.detect_holes(img)
        print(f"[INFO] detections: {detections}")

        # Filter and sort detections based on size
        detections = sorted(detections, key=lambda x: x["size"])
        labels = ["smallest", "small", "large", "largest"]

        # Stabilize the detections by averaging positions
        for i, detection in enumerate(detections):
            name = detection["name"]
            center = detection["center"]
            bbox = detection["bbox"]

            # Draw bounding box
            cv2.polylines(img, [bbox], True, (255, 0, 0), 3)

            # Get the top-left point of the bounding box for labeling
            top_left = tuple(bbox[0][0])  # Access the first point directly
            label_position = (top_left[0], top_left[1] - 10)  # Position label above the bounding box
            
            # Ensure the label index is within bounds
            label = labels[i] if i < len(labels) else f"torpedo_{i}"
            cv2.putText(img, label, label_position, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)


            if name in self.keypoints_descriptors:
                self.center_positions[name].append(center)

        # Control logic (keeping your existing logic intact)
        if self.step == 1 and any("torpedo_board" in detection["name"] for detection in detections):
            H = self.process_sift(self.reference_images["torpedo_board"], img, kp1, des1, threshold=0.65, window_viz=None)
            if H is not None:
                target = next(detection for detection in detections if detection["name"] == "torpedo_board")["center"]
                yaw, lateral, vertical, forward = self.move_forward(H, self.ref_shapes["torpedo_board"], target)
                print(f"[INFO] Moving to hole 1: yaw {yaw} lateral {lateral} vertical {vertical} forward {forward}")
                return yaw, lateral, vertical, forward

        return 0, 0, 0, 0


    def torpedo_board(self, img):
        """Align to torpedo board"""
        ref_img = self.reference_images["torpedo_board"]
        ref_shape = ref_img.shape
        kp1, des1 = self.keypoints_descriptors["torpedo_board"]
        H = self.process_sift(ref_img, img, kp1, des1, threshold=0.75, window_viz="Alignment")

        if H is None:
            print("[INFO] No Homography")
            return

        target = self.get_center(H, ref_shape, norm=True)
        yaw_required, yaw, dist = self.align_yaw(H, ref_shape)
        lateral_required, lateral = self.align_lateral(target)
        depth_required, vertical = self.align_depth(target)
        if not (yaw_required and lateral_required and depth_required):
            return {"yaw": yaw, "lateral": lateral, "vertical": vertical}

        # move forward
        if self.step == 1:
            print("[INFO] Forward")
            if dist > self.near_range:
                return {"forward": 0.5}
            if dist < self.near_range:
                self.step = 2
                self.aligned = False
        if self.step == 2:
            if not self.fired1:
                detections = self.detect_holes(img)
                for detection in detections:
                    name = detection["name"]
                    bbox = detection["bbox"]
                    cv2.polylines(img, [bbox], True, (0, 255, 0), 3, cv2.LINE_AA)
                if detections:
                    self.fired1 = True
                    return {"fire": 1, "fire_range": 2}
                return {"fire_range": 2}
            self.step = 3
            self.aligned = False
        if self.step == 3:
            if dist < self.firing_range:
                if not self.fired2:
                    detections = self.detect_holes(img)
                    for detection in detections:
                        name = detection["name"]
                        bbox = detection["bbox"]
                        cv2.polylines(img, [bbox], True, (0, 0, 255), 3, cv2.LINE_AA)
                    if detections:
                        self.fired2 = True
                        return {"fire": 2, "fire_range": 2}
                    return {"fire_range": 2}
            print("[INFO] Finished")
            
    def draw_torpedo(self, img, torpedo_positions):
        for position in torpedo_positions:
            x, y = position
            cv2.circle(img, (x, y), 10, (0, 0, 255), -1)
        return img
    
def main():
    parser = argparse.ArgumentParser(description="Run torpedo CV")
    parser.add_argument("--config", help="Configuration file", default="config.yaml")
    parser.add_argument("--video", help="Path to the video file for testing", default=None)
    args = parser.parse_args()

    config = {}
    cv = CV(**config)

    if args.video:
        # Use video file for testing
        video_path = args.video

        print(f"Video path: {video_path}")

        # Verify the path exists.
        if not os.path.exists(video_path):
            print(f"[ERROR] Video file not found: {video_path}")
            return
        
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"[ERROR] Unable to open video file: {video_path}")
            return
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[INFO] End of file.")
                break

            yaw, lateral, vertical, forward = cv.update_center(frame)
            print(f"[INFO] Yaw: {yaw}, Lateral: {lateral}, Vertical: {vertical}, Forward: {forward}")

            cv2.imshow("Frame", frame)
            time.sleep(0.05)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()
    else:
        # Use camera input
        cv.run()

if __name__ == "__main__":
    main()
