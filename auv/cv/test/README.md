## Description
This is the README for the auv/cv/test folder, a folder created to create CV (computer vision) code for the missions for 2024.As our platform at this point (6/05/2024) is not currently capable of executing a mission or even obtaining real-time detection output from our camera feed, our goal is to create scripts in which we can generate and test CV logic for each mission through training data which we have collected, so that when our capabilities are ready we can easily translate this code into mission-executable code.

However, there are some limitations, mainly that we only use OpenCV color thresholding and other OpenCV methods to detect objects, which limits the missions we can utilize this technique with considerably. Using YOLO models for detection has not been investigated yet, but will be in the coming days. The other limitation is the fact that we cannot use real-time data, and must resort to changing the "steps" (detailed) below and test each of our steps one at a time, without (most likely) being able to make 100% sure that our logic will work during a full mission.

## Table of Contents
- [Theory](#theory)
- [Template](#template)
- [Protocol](#protocol)

## Theory
Since the goal as stated is to create a testing platform for mission-specific CV logic, we will base the architecture of each script based on the actual CV script that we plan to use for each mission. The basic idea therefore, is to capture a training video, and on each frame run a detection method to detect the object of interest (for instance, a buoy). Once the object is detected, based on the position of the object in the frame, we determine what action to do next. Using this, and a stepwise system to keep track of what the AUV should do next, we output motion values, and potentially servo values (for the Torpedo, Bins, and Octagon missions), as well as the frame, with OpenCV annotations outlining the detected object, for testing purposes. See the "Template" section for a low-level view.

## Template
Please see the code inside this folder labeled "template_test_cv.py" for the template code. Attached to the code are all of the relevant comments and explanations.

## Protocol
To name a file that you wish to create, please use <mission_name>_test_cv.py, in all lowercase, underscores between words.

Inside each file, please put a header at the top in the following format:
"""
Description of script

Author: <Author_name>
"""

Additionally, please add comments as necessary throughout the code, but not too many as to clutter it. Use your own discretion, but the end goal is to have any programmer on our team quickly and easily understand the logic of your code, and to utilize aspects of it, assuming the programmer is at all familiar with subject matter.
