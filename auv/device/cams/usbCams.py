"""
Method for handling camera streams from USB cameras
"""

import os
import platform
import signal
import sys
import threading
import time

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from auv.device.cams import pyfakewebcam # For simulating a webcam by writing frames to a V4L2 virtual video device


class USBCamera:
    """
    Creates a camera stream from the USB cameras
    """

    def __init__(self, rospy, id, ogDevice, newDevice):
        self.IMG_W = 640
        self.IMG_H = 480

        self.rospy = rospy
        self.id = id
        self.frame = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(30)
        self.isKilled = True
        self.ogDevice = ogDevice
        self.newDevice = newDevice
        self.fake = pyfakewebcam.FakeWebcam(newDevice, self.IMG_W, self.IMG_H)
        self.pub = self.rospy.Publisher(f"/auv/camera/videoUSBRaw{str(id)}", Image, queue_size=10)
        self.rospy.Subscriber(f"/auv/camera/videoUSBOutput{str(id)}", Image, self.callbackMain)
        self.time = time.time()
        print(f"Camera ID {str(id)}: {ogDevice} is available at {newDevice}")

    def callbackMain(self, msg):
        if self.isKilled:
            return
        self.time = time.time()
        self.sendFakeFrame(self.br.imgmsg_to_cv2(msg))

    def sendFakeFrame(self, msg):
        try:
            # Run red object detection and draw bounding box if applicable
            frame = msg.copy()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([179, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            red_poles = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 1000:
                    x, y, w, h = cv2.boundingRect(cnt)
                    red_poles.append((x, y, w, h, area))
            if red_poles:
                red_poles.sort(key=lambda x: x[4], reverse=True)
                x, y, w, h, _ = red_poles[0]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.fake.schedule_frame(self.frame)
        except Exception as e:
            print(f"Camera {str(self.id)} Output Error, make sure running in correct python")
            print(e)

    def runner(self):
        while not self.rospy.is_shutdown() and not self.isKilled:
            try:
                ret, frame1 = self.cam.read()
                if ret:
                    new_frame = cv2.putText(frame1,  # The image on which to draw
                                            "DEBUG",  # The text string
                                            (300, 300),  # The bottom-left corner coordinates of the text
                                            cv2.FONT_HERSHEY_SIMPLEX,  # The font type
                                            1,  # The font scale factor
                                            (0,0,255),  # The text color (BGR format)
                                            thickness=1,  # The thickness of the text
                                            lineType=cv2.LINE_8,  # The line type (default is LINE_8)
                                            bottomLeftOrigin=False)
                    msg = self.br.cv2_to_imgmsg(new_frame)
                    self.pub.publish(msg)
                    if time.time() - self.time > 3:
                        self.sendFakeFrame(frame1)
                pass
            except Exception as e:
                print(f"Camera {str(self.id)} Input Error")
                print(e)
        self.loop_rate.sleep()

    def kill(self):
        if self.isKilled:
            return
        self.rospy.loginfo(f"Killing Camera {str(self.id)} Stream...")
        self.isKilled = True
        self.usbThread.join()
        self.cam.release()
        self.rospy.loginfo(f"Killed Camera {str(self.id)} Stream...")

    def start(self):
        self.cam = cv2.VideoCapture(self.ogDevice)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.IMG_W)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.IMG_H)
        self.isKilled = False
        self.rospy.loginfo(f"Starting Camera {str(self.id)} Stream...")
        self.usbThread = threading.Timer(0, self.runner)
        self.usbThread.daemon = True
        self.usbThread.start()