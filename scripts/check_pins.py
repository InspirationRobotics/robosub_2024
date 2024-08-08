import RPi.GPIO as GPIO
import time

# Pin Definitions (BCM numbering)
# input_pins = [7,
#               11, 12, 13, 15, 16, 18, 19,
#               21, 22, 22, 23, 24, 26, 28, 29,
#               31, 32, 33, 35, 36, 37, 38,
#               40]  # Replace with your specific pins

# Set up the GPIO channel
GPIO.setmode(GPIO.BOARD)

try:
    for pin in range(40):
        GPIO.setup(pin + 1, GPIO.IN)
except Exception:
    pass

try:
    while True:
        for pin in range(40):
            if GPIO.input(pin + 1):
                print(f"Pin {pin + 1} is HIGH")
            else:
                print(f"Pin {pin + 1} is LOW")
        time.sleep(1)
except Exception:
    pass
except KeyboardInterrupt:
    print("Program terminated")
finally:
    GPIO.cleanup()