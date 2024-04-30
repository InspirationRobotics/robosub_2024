"""
CV script for the PreQualification Maneuver.
"""

import time

import numpy as np
import cv2

class CV:
    """
    CV class for the PreQualification Maneuver.
    """

    camera = "/auv/camera/USBRaw0"

    def __init__(self, **config):
        """
        Initialize the PreQualification CV script.

        Args:
            config (dict): Dictionary containing the configuration of the devices on the sub.
        """

        self.step = 0
        self.shape = (480, 640)
        self.circleSide = "clockwise"

# Find the two sides of the gate
# Calculate the midpoint
# Yaw until aligned
# Strafe until aligned
# Move through the gate
# Keep moving until we see the marker, and it takes up a fair amount of the screen.
# Yaw in one direction, then move forward and slowly yaw in the other direction to circle the marker.
# After circling, keep moving forward slowly and adjust yaw then lateral to make exit the gate at the midpoint.

# NOTE: We can use yaw logic as follows:
    # First, yaw until both sides of the gate are in the frame
    # Then, make adjust laterally until the midpoint of the frame is aligned with the midpoint of the gate
    # Yaw again so that the sides of the gate are equal distanced from the side of the frame
    # Move laterally to fully align.

    def preProcessImage(self, frame):
        """
        Pre-processes the image by turning to gray scale then blurring the image.

        Args:
            frame (numpy.ndarray): Frame from the camera (this will be in the BGR color format).

        Returns:
            numpy.ndarray: Blurred Image
        """
        grayImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurredImage = cv2.GaussianBlur(grayImage, (5, 5), 0)
        return blurredImage
    
    def findVerticalSides(self, frame):
        """
        Finds the vertical sides of the gate.

        Args:
            frame (numpy.ndarray): Blurred Image (frame MUST be pre-processed in order for this function to work).

        Returns:
            list: A list of the detections of the vertical sides of the gate. Each entry in the list will be in the form (top left x-coordinate of detection,
                  top left y-coordinate of detection, width of detection, height of detection). There should hopefully only be two entries in the list.
        """

        edges = cv2.Canny(frame, 50, 150)
        contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        vertical_sides = []
        for contour in contours:
            perimeter = cv2.arcLength(contour, True)
            approximatedPolygon = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

            x, y, w, h = cv2.boundingRect(approximatedPolygon)

            if w < h:
                sideRatio = w/(h + 0.000001)
                if sideRatio <= 0.4:
                    vertical_sides.append(x, y, w, h)

        return vertical_sides
    
    def findMidpointOfGate(self, x1, x2, w1, w2):
        """
        Find the midpoint of the gate.

        Args:
            x1: Top left corner of the first detection.
            x2: Top left corner of the second detection.
            w1: Width of the first detection.
            w2: Width of the second detection.

        Returns:
            int: Midpoint of the gate (center x-coordinate).
        """
        midpointOfDetection1 = x1 + w1/2
        midpointOfDetection2 = x2 + w2/2
        return int((midpointOfDetection1 + midpointOfDetection2)/2)
    
    def alignWithMidpointCallback(self, detections):
        """
        Align with the midpoint of the gate, by yawing and strafing.

        Args:
            detections (list): Vertical side detections.

        Returns:
            tuple: Lateral command, yaw command.
        """
        yawTolerance = 20
        lateralTolerance = 20

        if len(detections) == 2:
                x1 = detections[0][0]
                w1 = detections[0][2]
                x2 = detections[1][0]
                w2 = detections[1][2]
                midpointOfGate = self.findMidpointOfGate(x1, x2, w1, w2)

        elif len(detections) == 0 or len(detections) == 1:
                yaw = 1
            
        if midpointOfGate != None:
                midpointOfFrame = self.shape[1]/2
                errorOfAlignment = midpointOfGate - midpointOfFrame

                if abs(errorOfAlignment) > lateralTolerance:
                    lateral = np.clip(errorOfAlignment * -3, -1, 1) # Adjust

                else:
                    lateral = 0

                # Find which detection is closer to the left side, right side.
                if x1 < x2:
                    leftCorner = x1
                    rightCorner = x2 + w2
                else:
                    leftCorner = x2
                    rightCorner = x1 + w1

                # If the difference between the two detections from the edge of the screen is not equal, yaw until they are.
                # If the left side shows up more, yaw clockwise, else yaw counterclockwise.
                if leftCorner - abs(rightCorner - self.shape[1]) > yawTolerance:
                    yaw = 0.5
                elif abs(rightCorner - self.shape[1]) - leftCorner > yawTolerance:
                    yaw = -0.5
                else:
                    yaw = 0

        return lateral, yaw
    
    def run(self, frame):
        """
        Run the PreQualification CV.

        Args:
            frame: Raw frame from the camera feed.

        Returns:
            dictionary, numpy.ndarray: {lateral movement command, forward movement command, yaw movment command, flag indicating whether the mission has ended}, visualized frame
        """

        processed_frame = self.preProcessImage(frame)
        detections = self.findVerticalSides(processed_frame)

        end = False
        lateral = 0
        forward = 0
        yaw = 0

        # Positive lateral value = move right, negative, move left.
        # Positive yaw value clockwise, negative counterclockwise.

        # If step 0, find and align with the midpoint of the gate.
        if self.step == 0:
            lateral, yaw = self.alignWithMidpointCallback(detections)
            if lateral or yaw != 0:
                pass
            else:
                self.step = 1
        
        # If step 1, move forward through the gate until we see the marker.
        if self.step == 1:
            pass

        # If step 2, move clockwise or counterclockwise around the marker, by changing yaw and forward values.

        # If step 3, move back through the gate.
                 
        
        return {"lateral" : lateral, "forward" : forward, "yaw" : yaw, "end" : end}, frame
                
