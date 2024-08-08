import RPi.GPIO as GPIO
import time

# Pin Definitions (BCM numbering)
input_pins = [1, 2, 3, 4, 5, 6, 7, 8, 9,
              10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
              20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
              30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
              40]  # Replace with your specific pins

# Set up the GPIO channel
GPIO.setmode(GPIO.BOARD)

try:
    for pin in input_pins:
        GPIO.setup(pin, GPIO.IN)
except Exception:
    pass

try:
    while True:
        for pin in input_pins:
            if GPIO.input(pin):
                print(f"Pin {pin} is HIGH")
            else:
                print(f"Pin {pin} is LOW")
        time.sleep(1)
except Exception:
    pass
except KeyboardInterrupt:
    print("Program terminated")
finally:
    GPIO.cleanup()