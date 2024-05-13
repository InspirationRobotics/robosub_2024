# TODO: See whether this works when connected with the sub.

import cv2
from auv.utils import deviceHelper

camera = "forwardOak"
port = deviceHelper.dataFromConfig(camera)
print(port)
cap = cv2.VideoCapture(port)

while True:
    # Grab a frame from the video object.
    ret, frame = cap.read()
    if not ret:
        break

    # Show the frame for debugging purposes.
    cv2.imshow("frame", frame)

    # If key 'q' pressed, then exit the program.
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    
    # These two lines are absolutely necessary; this cleans up the capture system, so the file does not become corrupted.
cv2.vid.release()
cv2.destroyAllWindows()
