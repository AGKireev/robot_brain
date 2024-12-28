import os
import sys
import time
import logging

# Add the parent directory of 'server' to sys.path
script_dir = os.path.realpath(os.path.dirname(__file__))
sys.path.append(os.path.abspath(os.path.join(script_dir, '..', 'server')))

import move

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


if __name__ == '__main__':
    logger.info("move: __main__")
    step = 1
    move_stu = 1
    try:
        # Walk test
        # while 1:
        #     move(step, 35, 'no')
        #     step += 1
        #     if step > 4:
        #         step = 1
        #     time.sleep(0.08)

        # Dove test
        while 1:
            move.dove(1,-35,0.01,17,'no')
            move.dove(2,-35,0.01,17,'no')
            move.dove(3,-35,0.01,17,'no')
            move.dove(4,-35,0.01,17,'no')

        # Steady test
        # steady_X()
        # while 1:
        #     steady()
        #     time.sleep(0.02)

        '''
        for i in range(0,9):
            look_left()
            time.sleep(1)
        for i in range(0,16):
            look_right()
            time.sleep(1)	
        time.sleep(1)
        look_home()
        '''

        #pwm.set_all_pwm(0,0)
        #pwm.set_all_pwm(0, 300)
        #time.sleep(10)

    except KeyboardInterrupt:
        # set all to safe position
        for i in range(16):
            sc.set_servo_pwm(i, 300)
        time.sleep(1)
        # clean_all()
