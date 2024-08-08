import RPi.GPIO as GPIO
import time

# Pin Definitions (BCM numbering)
# input_pins = [4, 17, 27, 22]  # Replace with your specific pins

# Set up the GPIO channel
GPIO.setmode(GPIO.BOARD)
# for pin in range(40):
#     GPIO.setup(pin + 1, GPIO.IN)

# try:
#     while True:
#         for pin in range(40):
#             if GPIO.input(pin + 1):
#                 print(f"Pin {pin} is HIGH")
#             else:
#                 print(f"Pin {pin} is LOW")
#         time.sleep(1)

# except KeyboardInterrupt:
#     print("Program terminated")
# finally:
#     GPIO.cleanup()




GPIO.setup(19, GPIO.IN)

if GPIO.input(19):
    print(f"Pin is high, input {GPIO.input(19)}")
else:
    print(f"Pin is low, input {GPIO.input(19)}")