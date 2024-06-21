import RPi.GPIO as GPIO
import time

if __name__ == "__main__":
    # Set up GPIO pin, default
    GPIO.setmode(GPIO.BCM)
    fet_pin = 26
    GPIO.setup(fet_pin, GPIO.OUT)
    GPIO.output(fet_pin, GPIO.LOW)

    # output high         
    input("Press key to turn relay on")
    GPIO.output(fet_pin, GPIO.HIGH)
    print("pin on")

    # Shutdown the bus
    input("Press any key to shut down.")
    print("Shutting down, finishing script.")
    GPIO.output(fet_pin, GPIO.LOW)

    print("done")