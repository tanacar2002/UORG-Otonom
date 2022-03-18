#!/usr/bin/python3

import RPi.GPIO as GPIO
import time
PIN = 12

GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN, GPIO.OUT)
try:
    while True:
        GPIO.output(PIN, False)
        time.sleep(0.5)
        GPIO.output(PIN, True)
        time.sleep(0.5)
finally:
    GPIO.cleanup()