"""
Template method for capturing a camera stream, and writing the stream to a video file. 

The only things that really should need to be adjusted have a comment "adjust".
"""

# TODO: Switch the settings back to the way they are supposed to be for the sub.

import time

import cv2
import numpy as np 


class CV:
    """
    Template CV class, DO NOT change the name of the class. Doing so would mess up all of the backend CV script execution files.
    """

    def __init__(self, **config):
        """
        Initialize the CV class. 
        """

        # Set the camera to get the camera stream from.
        self.camera = "/auv/camera/videoUSBRaw0" # Adjust based on the mission.

        # Setting frame dimensions.
        self.frame_width = 640
        self.frame_height = 480
        self.size = (self.frame_width, self.frame_height)

        self.pathToVideo = '../../../testing_data/test_video.mp4' # Adjust as necessary.
        self.codec = 'MJPG' # Change from "MJPG" to "mp4v" if capturing from a Windows camera.

        self.vid =  cv2.VideoWriter(self.pathToVideo,
                          cv2.VideoWriter_fourcc(*self.codec), 
                          30, self.size)
        
        
    def run(self, frame, target, oakd_data):
        """
        Capture the video. Only "frame" is technically necessary, "target" and "oakd_data" are just there for consistency with the other CV scripts.

        Args:
            frame: Current frame from camera stream.
            target: Target to look for.
            oakd_data: List of detections from the ML model; this is only applicable for the OAK-D camera streams.

        Returns:
            dictionary, numpy.ndarray: {Lateral motion command, forward motion command, end flag, video buffer}, frame

        NOTE: There is not technically anything necessary to return, as the file will be run independently. However, the returns are there to maintain 
        consistency with the other CV scripts, like the args.
        """

        # Write the frame -- this will write the current frame to the video file.
        self.vid.write(frame)

        return {"lateral": 0, "forward": 0, "end": False, "vid": self.vid}, frame
    


if __name__ == "__main__":
    """
    Type "python3 -m auv.cv.data.auto_cam_capture_cv.py" in order to run this script.
    This should be all you need in order to obtain testing data. 
    """
    from auv.utils import deviceHelper

    camera = "forwardOak" # Adjust
    port = deviceHelper.dataFromConfig(camera)

    # Create a CV object with arguments.
    cv = CV()

    # Initializing camera capture object.
    cap = cv2.VideoCapture(port) # Switch "port" to "0" to get input from a Windows camera.

    while True:
        # Grab a frame from the video object.
        ret, frame = cap.read()
        if not ret:
            break

        # Run the CV processing.
        result = cv.run(frame, None, None)
    
        # Show the frame for debugging purposes.
        cv2.imshow("frame", frame)

        # If key 'q' pressed, then exit the program.
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    
    # These two lines are absolutely necessary; this cleans up the capture system, so the file does not become corrupted.
    cap.release()
    cv.vid.release()
    cv2.destroyAllWindows()