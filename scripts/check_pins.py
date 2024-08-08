import RPi.GPIO as GPIO
import time

# Pin Definitions (BCM numbering)
input_pins = [22]  # Replace with your specific pins

# Set up the GPIO channel
GPIO.setmode(GPIO.BOARD)

for pin in input_pins:
    try:
        GPIO.setup(pin, GPIO.IN)
    except Exception:
        print("I threw an exception!")

while True:
    for pin in input_pins:
        try:
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
