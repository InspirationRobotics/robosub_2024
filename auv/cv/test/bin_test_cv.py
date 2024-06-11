"""Bin mission CV code"""

import time
import cv2
import numpy as np
import shapely

class CV: 
    """
    CV class for Bin mission. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """
  
    camera = "/auv/camera/videoOAKdRawBottom"
    model = "bin"
    
    def __init__(self, **config):
        """Initialize CV class"""
      
        self.viz_frame = None
        self.error_buffer = []
      
        print("[INFO] Bin CV Init")

    def get_bbox_center(self, detection):
        x1 = int(detection.xmin)
        x2 = int(detection.xmax)
        y1 = int(detection.ymin)
        y2 = int(detection.ymax)

        return ((x1 + x2) // 2, (y1 + y2) // 2)

    def run(self, frame, target, oakd_data):
        """
        Run the CV script.

        Args:
            frame: The frame from the camera stream
            target: This can be any type of information, for example, the object to look for
            detections: This only applies to OAK-D cameras; this is the list of detections from the ML model output

        Here should be all the code required to run the CV.
        This could be a loop, grabbing frames using ROS, etc.

        Returns:
            dictionary, visualized frame: {motion commands/flags for servos and other indication flags}, visualized frame
        """

        forward = 0
        lateral = 0
        yaw = 0

        height, width, _ = frame.shape

        tolerance = 0.1
        maxConfidence = 0
        target_color = None
        red = None
        blue = None
        red_confidence = 0
        blue_confidence = 0
        bin = []
        colors = []

        # measured offset from the footage
        target_pixel = (190, 300)

        if len(oakd_data) == 0:
            return {"forward": 0.8}, frame

        for detection in oakd_data:
            x1 = int(detection.xmin)
            x2 = int(detection.xmax)
            y1 = int(detection.ymin)
            y2 = int(detection.ymax)

            if "blue" in detection.label and detection.confidence > blue_confidence:
                blue = detection
                blue_confidence = detection.confidence

            elif "red" in detection.label and detection.confidence > red_confidence:
                red = detection
                red_confidence = detection.confidence

        if "red" in target:
            target_color = red

        elif "blue" in target:
            target_color = blue

        approach = False
        if target_color is None:
            if red is None and blue is None:
                if len(bin) == 0:
                    return {"forward": 0.8}, frame
                else:
                    # average the positions of the bin
                    avg_center = np.mean([self.get_bbox_center(b) for b in bin], axis=0)
                    target_color_center = (int(avg_center[0]), int(avg_center[1]))
                    approach = True
            elif red is None:
                target_color = blue
            elif blue is None:
                target_color = red

        if not approach:
            pass

        cv2.rectangle(
            frame,
            (int(target_color.xmin), int(target_color.ymin)),
            (int(target_color.xmax), int(target_color.ymax)),
            (255, 0, 0),
            2,
        )

        x_error = (target_color_center[0] - target_pixel[0]) / width
        y_error = (target_pixel[1] - target_color_center[1]) / height

        # apply a gain and clip the values
        lateral = np.clip(x_error * 3.5, -1, 1)
        forward = np.clip(y_error * 3.5, -1, 1)

        if len(self.error_buffer) > 30:
            self.error_buffer.pop(0)
        self.error_buffer.append((x_error + y_error) / 2)
        avg_error = np.mean(np.linalg.norm(self.error_buffer, axis=1))

        if avg_error < tolerance and len(self.error_buffer) == 30:
            aligned = True

        # TODO (low priority): Remove colors for each bin
        return {"lateral": lateral, "forward": forward, "aligned": aligned}, frame

if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.template_cv"
    video_root_path = "" # Computer path through the training data folder.
    mission_name = "Ocean Temperatures/" # Mission folder
    video_name = "" # Specified video
    video_path = os.bin.join(video_root_path, mission_name, video_name)

    # For testing
    print(f"Video path: {video_path}")
    
    # Create a CV object with arguments
    cv = CV()
    
    # here you can for example initialize your camera, etc
    cap = cv2.VideoCapture("../../testing_data/")

    while True:
        # grab a frame
        ret, frame = cap.read()
        if not ret:
            break

        # run the cv
        result = cv.run(frame, "some_info", None)
    
        # do something with the result
        print(f"[INFO] {result}")
    
        # debug the frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
