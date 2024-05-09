"""
CV template class. Copy this for each mission, and modify it accordingly.
"""

# Import what you need from within the package.

import time

import cv2
import numpy as np

class CV:
    """
    Template CV class. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """

    # Camera to get the camera stream from.
    camera = "/auv/camera/videoUSBRaw0" 

    def __init__(self, **config):
        """
        Initialize the CV class. 
        Setup/attributes here will contain everything needed for the run function.
        
        Args:
            config: Dictionary that contains the configuration of the devices on the sub.
        """

        # This is an example of getting a stored value from the configuration. 
        self.this_is_a_variable = config.get("this_is_a_variable", 42)

        print("[INFO] Template CV init")

    def run(self, frame, target, detections):
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
        print("[INFO] Template CV run")

        # Continuously return motion commands, the state of the mission, and the visualized frame.
        return {"lateral": 0, "forward": 0, "end": False}, frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly.
    # It is here for testing purposes.
    # You can run this file independently using: "python -m auv.cv.template_cv".

    # Create a CV object with arguments
    cv = CV()

    # Here you can initialize your camera, etc.

    # Capture the video object for processing
    cap = cv2.VideoCapture(0)

    while True:
        # Grab and read a frame from the video object.
        ret, frame = cap.read()
        if not ret:
            break

        # Run the CV script.
        result = cv.run(frame, "some_info", None)

        # Do something with the result. 
        print(f"[INFO] {result}")

        # Debug the visualized frame.
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
