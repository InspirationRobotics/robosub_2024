# TODO: Pretty much everything.

import cv2

cap = cv2.VideoCapture(r"C:\Users\netwo\OneDrive\Team Inspiration\RoboSub 2024\robosub_2024\testing_data\AutoVideo.mp4")

_, frame = cap.read()
cv2.imshow("frame", frame)

# def split_video_into_frames(video_path, output_folder):
#     """
#     Split a given video at a given path into frames inside a folder.

#     Args:
#         video_path (str): Path to the video.
#         output_folder (str): The name of the folder that will contain the video frames.
#     """
#     # Open the video file.
#     vidcap = cv2.VideoCapture(video_path)

#     # Create the output folder if it doesn't exist.
#     import os
#     if not os.path.exists(output_folder):
#         os.makedirs(output_folder)

#     # Read the video frame by frame.
#     success, image = vidcap.read()

#     count = 0
#     while success:
#         # Write the current frame to a JPEG file.
#         cv2.imwrite(os.path.join(output_folder, "frame%d.jpg" % count), image)
#         success, image = vidcap.read()
#         print('Frame %d: Success' % count)
#         count += 1

#     print("Frames extraction completed.")

# video_path = r"C:\Users\netwo\OneDrive\Team Inspiration\RoboSub 2024\robosub_2024\testing_data\AutoVideo.mp4"
# output_folder = "AutoVideo_output_folder"
# split_video_into_frames(video_path, output_folder)