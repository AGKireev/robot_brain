"""
This script controls the single LED switches on the HAT
connected to 3 GPIO pins.
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)

"""

print("Unused logic, must be refactored!")
quit()

"""
MOVED FROM main.py:

	
# LED switch setup (not used for now)
	# switch.switchSetup()
	# switch.set_all_switch_off()	
"""

import time
import logging
import RPi.GPIO as GPIO

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def switchSetup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(5, GPIO.OUT)
    GPIO.setup(6, GPIO.OUT)
    GPIO.setup(13, GPIO.OUT)

def switch(port, status):
    if port == 1:
        if status == 1:
            GPIO.output(5, GPIO.HIGH)
        elif status == 0:
            GPIO.output(5,GPIO.LOW)
        else:
            pass
    elif port == 2:
        if status == 1:
            GPIO.output(6, GPIO.HIGH)
        elif status == 0:
            GPIO.output(6,GPIO.LOW)
        else:
            pass
    elif port == 3:
        if status == 1:
            GPIO.output(13, GPIO.HIGH)
        elif status == 0:
            GPIO.output(13,GPIO.LOW)
        else:
            pass
    else:
        logger.error('switch: Wrong Command: Example--switch(3, 1)->to switch on port3')

def set_all_switch_off():
    switch(1,0)
    switch(2,0)
    switch(3,0)

if __name__ == '__main__':
    switchSetup()
    try:
        while 1:
            switch(1,1)
            switch(2,1)
            switch(3,1)
            logger.info("switch: Light on....")
            time.sleep(1)
            set_all_switch_off()
            logger.info("switch: Light off....")
            time.sleep(1)
    except:
        set_all_switch_off()
    