"""
CV code for the gate mission. 

Should be relatively straightforward: get the detection data from the ML model, then align with the midpoint of the given detection, assuming the 
confidence of the detection is high enough.
"""

# Import what you need from within the package.

import time

import cv2
import numpy as np

class CV:
    """
    CV class for the Gate mission. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.

    Attributes:
        self.shape (tuple): (height, width) of the frame.
    """

    # Camera to get the camera stream from.
    camera = "/auv/camera/videoUSBRaw0" 
    model = "buoy"

    def __init__(self, **config):
        """
        Initialize the CV class. 
        Setup/attributes here will contain everything needed for the run function.
        
        Args:
            config: Dictionary that contains the configuration of the devices on the sub.
        """

        self.shape = (480, 640)

        print("[INFO] Buoy CV init")

    def run(self, frame, target, detections):
        """
        Run the CV script.

        Args:
            frame: The frame from the camera stream
            target (str): The side of the gate to choose, either blue or red. 
            detections (list): This only applies to OAK-D cameras; this is the list of detections from the ML model output

        Here should be all the code required to run the CV.
        This could be a loop, grabbing frames using ROS, etc.

        Returns:
            dictionary, visualized frame: {lateral motion command, forward motion command, yaw motion command, end flag}, visualized frame
        """
        print("[INFO] Buoy CV run")

        forward = 0
        lateral = 0
        yaw = 0
        end = 0

        detections_info = repr(detections)
        # detection_str = str(detections)
        # detection_label = detections.label

        print(f"{forward} {lateral} {yaw} {end} {detections_info}")
        # targetDetection = None
        # targetDetected = False
        # step = 0
        # end = False

        # if len(detections) < 2:
        #     yaw = 1

        # if len(detections) == 2:
        #     for detection in detections:
        #         if target in detection.label:
        #             targetDetection = detection

        # if targetDetection is not None:
        #     if targetDetection.confidence > 0.5:
        #         targetDetected = True
        #     else:
        #         targetDetected = False
        
        # if targetDetected == True:
        #     step = 1

        # if step == 1:
        #     # Align with the target.
        #     lateral = 0
        #     forward = 0
        
        # if end == True:
        #     print("Ending mission")
                    
        # # If the length of detections is not equal to two, then yaw to find the detections.
        # # Find the detection with the target label.
        # # If the confidence of the detection is high enough, then obtain the x and y coordinates of the detection on the frame.
        # # Align so that the x midpoint is aligned with the midpoint of the frame, and the y coordinate should be significantly higher than the y midpoint.
        # # End.

        # Continuously return motion commands, the state of the mission, and the visualized frame.
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": end}, frame


if __name__ == "__main__":
    cv = CV()
    data, frame = cv.run(None, None, None)
    print(data)

    # # This is the code that will be executed if you run this file directly.
    # # It is here for testing purposes.
    # # You can run this file independently using: "python -m auv.cv.template_cv".

    # # Create a CV object with arguments
    # cv = CV()

    # # Here you can initialize your camera, etc.

    # # Capture the video object for processing
    # cap = cv2.VideoCapture(0)

    # while True:
    #     # Grab and read a frame from the video object.
    #     ret, frame = cap.read()
    #     if not ret:
    #         break

    #     # Run the CV script.
    #     result = cv.run(frame, "some_info", None)

    #     # Do something with the result. 
    #     print(f"[INFO] {result}")

    #     # Debug the visualized frame.
    #     cv2.imshow("frame", frame)
    #     if cv2.waitKey(1) & 0xFF == ord("q"):
    #         break