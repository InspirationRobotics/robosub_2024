import cv2
import time

import os

# Import the video 
# Read each frame
# Detect the buoy through color thresholding
# Return motion values

class CV:
    def __init__(self):
        pass

    def detect_buoy(frame):
        pass
    def run(frame):
        pass

if __name__ == "__main__":
    video_root_path = "/home/kc/Desktop/Team Inspiration/RoboSub 2024/Training Data/"
    mission_name = "Buoy/"
    video_name = "Buoy Video 1.mp4"
    video_path = os.path.join(video_root_path, mission_name, video_name)
    print(f"Video path: {video_path}")
    #cv = CV()

    if not os.path.exists(video_path):
        print(f"[ERROR] Video file not found {video_path}")
    else:
        cap = cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"[ERROR] Unable to open video file: {video_path}")
        else:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("[INFO] End of file.")
                    break
                cv2.imshow("frame", frame)
                time.sleep(0.01)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break