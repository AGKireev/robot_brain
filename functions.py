import time
import threading
import logging
from mpu6050 import mpu6050

from system import config
from system.kalman_filter import KalmanFilter
from servo import base

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

logger.info('Functions: starting..')

# Initialize the servo control
logger.info('Functions: initializing servo')
scGear = base.ServoCtrl()

# Initialize Kalman filter for X axis
kalman_filter_X = KalmanFilter(0.01, 0.1)

# Initialize MPU6050 sensor
logger.info('Functions: initializing MPU6050')
MPU_connection = 1
try:
    sensor = mpu6050(0x68)
    logger.info('mpu6050 connected, PT MODE ON')
except:
    MPU_connection = 0
    logger.info('mpu6050 disconnected, ARM MODE ON')

# Initialize PWM values and directions
pwm_config = config.read("pwm")

pwm0_direction = 1
pwm0_init = pwm_config["init_pwm0"]
pwm0_max = 520
pwm0_min = 100
pwm0_pos = pwm0_init

pwm1_direction = 1
pwm1_init = pwm_config["init_pwm1"]
pwm1_max = 520
pwm1_min = 100
pwm1_pos = pwm1_init

pwm2_direction = 1
pwm2_init = pwm_config["init_pwm2"]
pwm2_max = 520
pwm2_min = 100
pwm2_pos = pwm2_init

def pwmGenOut(angleInput):
    '''
    Generate PWM output from angle input.
    '''
    return int(round(23 / 9 * angleInput))

class Functions(threading.Thread):
    def __init__(self, *args, **kwargs):
        logger.info('Functions: __init__')
        self.functionMode = 'none'
        self.steadyGoal = 0

        self.scanNum = 3
        self.scanList = [0, 0, 0]
        self.scanPos = 1
        self.scanDir = 1
        self.rangeKeep = 0.7
        self.scanRange = 100
        self.scanServo = 1
        self.turnServo = 2
        self.turnWiggle = 200

        super(Functions, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()

    def radarScan(self):
        '''
        Perform a radar scan using the servo.
        '''
        logger.info('Functions: radarScan')

        global pwm0_pos
        scan_speed = 3
        result = []

        if pwm0_direction:
            pwm0_pos = pwm0_max
            scGear.set_servo_pwm(12, pwm0_pos)
            time.sleep(0.8)

            while pwm0_pos > pwm0_min:
                pwm0_pos -= scan_speed
                scGear.set_servo_pwm(12, pwm0_pos)
        else:
            pwm0_pos = pwm0_min
            scGear.set_servo_pwm(12, pwm0_pos)
            time.sleep(0.8)

            while pwm0_pos < pwm0_max:
                pwm0_pos += scan_speed
                scGear.set_servo_pwm(12, pwm0_pos)
        scGear.set_servo_pwm(12, pwm0_init)
        return result

    def pause(self):
        '''
        Pause the current function.
        '''
        logger.info('Functions: pause')
        self.functionMode = 'none'
        self.__flag.clear()

    def resume(self):
        '''
        Resume the current function.
        '''
        logger.info('Functions: resume')
        self.__flag.set()

    def automatic(self):
        '''
        Set the function mode to automatic.
        '''
        logger.info('Functions: automatic')
        self.functionMode = 'Automatic'
        self.resume()

    def trackLine(self):
        '''
        Set the function mode to track line.
        '''
        logger.info('Functions: trackLine')
        self.functionMode = 'trackLine'
        self.resume()

    def keepDistance(self):
        '''
        Set the function mode to keep distance.
        '''
        logger.info('Functions: keepDistance')
        self.functionMode = 'keepDistance'
        self.resume()

    def steady(self, goalPos):
        '''
        Set the function mode to steady.
        '''
        logger.info('Functions: steady')
        self.functionMode = 'Steady'
        self.steadyGoal = goalPos
        self.resume()

    def speech(self):
        '''
        Set the function mode to speech recognition processing.
        '''
        logger.info('Functions: speech')
        self.functionMode = 'speechRecProcessing'
        self.resume()

    def trackLineProcessing(self):
        '''
        Process the track line function.
        '''
        logger.info('Functions: trackLineProcessing')
        pass

    def automaticProcessing(self):
        '''
        Process the automatic function.
        '''
        logger.info('Functions: automaticProcessing')
        pass

    def steadyProcessing(self):
        '''
        Process the steady function.
        '''
        logger.info('Functions: steadyProcessing')
        pass

    def speechRecProcessing(self):
        '''
        Process the speech recognition function.
        '''
        logger.info('Functions: speechRecProcessing')
        pass

    def keepDisProcessing(self):
        '''
        Process the keep distance function.
        '''
        logger.info('Functions: keepDisProcessing')
        pass

    def functionGoing(self):
        '''
        Execute the current function mode.
        '''
        logger.info(f'Functions: functionGoing in {self.functionMode}')
        if self.functionMode == 'none':
            self.pause()
        elif self.functionMode == 'Automatic':
            self.automaticProcessing()
        elif self.functionMode == 'Steady':
            self.steadyProcessing()
        elif self.functionMode == 'trackLine':
            self.trackLineProcessing()
        elif self.functionMode == 'speechRecProcessing':
            self.speechRecProcessing()
        elif self.functionMode == 'keepDistance':
            self.keepDisProcessing()

    def run(self):
        '''
        Run the function thread.
        '''
        logger.info('Functions: run')
        while True:
            self.__flag.wait()
            self.functionGoing()
            pass

if __name__ == '__main__':
    pass
    # fuc=Functions()
    # fuc.radarScan()
    # fuc.start()
    # fuc.automatic()
    # # fuc.steady(300)
    # time.sleep(30)
    # fuc.pause()
    # time.sleep(1)
    # move.move(80, 'no', 'no', 0.5)
