import cv2

path = "/home/kc/Desktop/Testing Data/Buoy/Test Video 1.mp4"
video = cv2.VideoCapture(path)

if not video.isOpened():
    print("Unable to open video.")

output_folder = "Test Video 1 Frames"
import os
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

frame_count = 0

while True:
    ret, frame = video.read()
    if not ret:
        break

    frame_count += 1
    frame_path = os.path.join(output_folder, f'frame_{frame_count:04d}.jpg')
    cv2.imwrite(frame_path, frame)
    print(frame_count)

video.release()
cv2.destroyAllWindows()
print("Successful extraction of frames.")