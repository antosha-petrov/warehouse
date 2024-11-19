import sys
import time

if sys.platform == 'win32':
    import mock.RPi.GPIO as GPIO
else:
    import RPi.GPIO as GPIO

class LenSensor:
    def __init__(self, trig_pin=23, echo_pin=24):
        self.TRIG = trig_pin
        self.ECHO = echo_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)

        GPIO.output(self.TRIG, False)
        time.sleep(2)

    def measure_distance(self):
        GPIO.output(self.TRIG, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, False)

        pulse_start = None
        pulse_end = None

        while GPIO.input(self.ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(self.ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)
        return distance