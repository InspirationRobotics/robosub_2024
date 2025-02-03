This is the README for the data capturing folder.

The goal of this folder is to capture testing data from our mock task elements, in order to train our YOLO models properly.

auto_cam_capture_cv.py is our template file for capturing a camera feed from the sub, and placing the data into a video file (.mp4) format.

The videos should be located in the "testing_data" folder in the GitHub repository. These files should ONLY be there temporarily. Once we have the data, 
we should move the file into something like a Google Drive to save space. From there we can easily download the file to our individual computers, and change the format using OpenCV from a video file to a folder containing individual frames that we can use to label and train the YOLO model.

