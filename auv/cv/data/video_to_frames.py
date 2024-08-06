import cv2
import os

video_root_path = r"C:\Users\netwo\Downloads"
mission_name = r"Buoy_Competition"
video_name = r"Buoy Comp Video 16.mp4"
video_path = os.path.join(video_root_path, mission_name, video_name)

video = cv2.VideoCapture(video_path)

if not video.isOpened():
    print("Unable to open video.")

output_folder = f"{video_name} Frames"

folder_path = os.path.join(video_root_path, mission_name, output_folder)

if not os.path.exists(folder_path):
    os.makedirs(folder_path)

frame_count = 0

while True:
    ret, frame = video.read()
    if not ret:
        break

    frame_count += 1
    # if frame_count > 2:
    #     break
    frame_path = os.path.join(folder_path, f'frame_{frame_count:04d}.jpg')
    cv2.imwrite(frame_path, frame)
    print(frame_count)

video.release()
cv2.destroyAllWindows()
print("Successful extraction of frames.")