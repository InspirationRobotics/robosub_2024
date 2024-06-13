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
        # Config is a way of passing in an argument to indicate to the logic what actions to take. Take a look at 
        # buoy_test_cv.py for an example.
        self.shape = (640, 480)

        # Switcher variables which can be used as needed to switch states.
        self.aligned = False
        self.detected = False

        self.config = config 
        self.step = 0 # Step counter variable.

        self.end = False # End variable to denote when the mission has finished.

        print("[INFO] Bin CV Init")
        
        # Add variables as needed below.
        self.viz_frame = None
        self.error_buffer = []
      
        print("[INFO] Bin CV Init")

    def get_bbox_center(self, detection):
        x1 = int(detection.xmin)
        x2 = int(detection.xmax)
        y1 = int(detection.ymin)
        y2 = int(detection.ymax)

        return ((x1 + x2) // 2, (y1 + y2) // 2)

    def detect_red(self, frame):
        """
        Uses HSV color space and masking to detect a red object.
        """
        detected = False
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red_mask_1 = np.array([0, 120, 150])
        upper_red_mask_1 = np.array([10, 255, 255])
        lower_red_range_mask = cv2.inRange(hsv, lower_red_mask_1, upper_red_mask_1)

        lower_red_mask_2 = np.array([170, 120, 150])
        upper_red_mask_2 = np.array([180, 255, 255])
        upper_red_range_mask = cv2.inRange(hsv, lower_red_mask_2, upper_red_mask_2)

        mask = lower_red_range_mask + upper_red_range_mask

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key = cv2.contourArea)

            if cv2.contourArea(largest_contour) > 0:
                x, y, w, h = cv2.boundingRect(largest_contour)
                detected = True
                return {"status": detected, "xmin" : x, "xmax" : (x + w), "ymin" : (y), "ymax" : (y + h)}
        
        return {"status": detected, "xmin" : None, "xmax" : None, "ymin" : None, "ymax" : None}

    def detect_blue(self, frame):
        """
        Uses HSV color space and masking to detect a blue object.
        """
        detected = false
        hsv = cv2.cvtColor(frame, cv2.COLOR_ )

        lower_blue_mask_1 = np.array(p
        upper_blue_mask_1 = np.array
        lower_blue_range_mask = cv2.inRange(hsv, lower_blue_mask_1, lower_blue_mask_1)

        lower_blue_mask_2 = np.array(p
        upper_blue_mask_2 = np.array
        upper_blue_range_mask = cv2.inRange(hsv, lower_blue_mask_2, lower_blue_mask_2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key = cv2.contourArea)

            if cv2.contourArea(largest_contour) > 0:
                x, y, w, h = cv2.boundingRect(largest_contour)
                detected = True
                return {"status": detected, "xmin" : x, "xmax" : (x + w), "ymin" : (y), "ymax" : (y + h)}
        
        return {"status": detected, "xmin" : None, "xmax" : None, "ymin" : None, "ymax" : None}

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

# This if statement is just saying what to do if this script is run directly. 
if __name__ == "__main__":
    # Example of how to obtain a training video. Make sure to follow this template when capturing your own video, in case 
    # another team member needs to run this code on his/her device. 
    
    # NOTE: When downloading the training data, the training data folder itself, which contains all of the data.
    video_root_path = "" # Computer path through the training data folder.
    mission_name = "/" # Mission folder
    video_name = "" # Specified video
    video_path = os.path.join(video_root_path, mission_name, video_name)

    # For testing
    print(f"Video path: {video_path}")

    # Initialize an instance of the class.
    cv = CV("Blue")

    # Verify the path exists.
    if not os.path.exists(video_path):
        print(f"[ERROR] Video file not found {video_path}")
    else:
        # Capture the video object (basically access the specified video) at the specified path.
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"[ERROR] Unable to open video file: {video_path}")
        else:
            while True:
                # Access each frame of the video.
                ret, frame = cap.read()
                if not ret:
                    print("[INFO] End of file.")
                    break

                # Run the run function on the frame, and get back the relevant results.
                motion_values, viz_frame = cv.run(frame)
                if viz_frame is not None:
                    cv2.imshow("frame", viz_frame)
                else:
                    print("[ERROR] Unable to display frame.")

                # For testing purposes.
                print(f"Motion: {motion_values}")
                
                time.sleep(0.05)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
