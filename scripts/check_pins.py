import RPi.GPIO as GPIO
import time

# Pin Definitions (BCM numbering)
input_pins = [7,
              10, 11, 12, 13, 15, 16, 18, 19,
              21, 22, 22, 23, 24, 26, 27, 28, 29,
              31, 32, 33, 35, 36, 37, 38,
              40]  # Replace with your specific pins

# Set up the GPIO channel
GPIO.setmode(GPIO.BOARD)
for pin in input_pins:
    GPIO.setup(pin, GPIO.IN)

try:
    while True:
        for pin in input_pins:
            if GPIO.input(pin):
                print(f"Pin {pin} is HIGH")
            else:
                print(f"Pin {pin} is LOW")
        time.sleep(1)

except KeyboardInterrupt:
    print("Program terminated")
finally:
    GPIO.cleanup()