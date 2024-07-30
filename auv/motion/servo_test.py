import rospy
import time

from auv.motion.servo import Servo

servo = Servo()

time.sleep(5)

print("[INFO] Dropping ball")

servo.drop()

time.sleep(2)

print("[INFO] Servo test terminated.")
