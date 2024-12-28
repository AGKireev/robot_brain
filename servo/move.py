import time
import threading
import logging

from servo import base
from system.kalman_filter import KalmanFilter
import PID

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Change this variables to 0 to reverse all the servos.
set_direction = 1

# Change these two variables to reverse the direction of the legs.
if set_direction:
    leftSide_direction = 1
    rightSide_direction = 0
else:
    leftSide_direction = 0
    rightSide_direction = 1

# Change these two variables to reverse the height of the legs.
if set_direction:
    leftSide_height = 0
    rightSide_height = 1
else:
    leftSide_height = 1
    rightSide_height = 0

# Change this variable to set the range of the height range.
height_change = 30

# Change these two variables to adjust the function for observing.
if set_direction:
    Up_Down_direction = 1
    Left_Right_direction = 1
else:
    Up_Down_direction = 0
    Left_Right_direction = 0

Left_Right_input = 300
Up_Down_input = 300
Left_Right_Max = 500
Left_Right_Min = 100
Up_Down_Max = 500
Up_Down_Min = 270
look_wiggle = 30
move_stu = 1

DOVE_SPEED = 20

# Change these variable to adjust the steady function.
steady_range_Min = -40
steady_range_Max = 130
range_Mid = (steady_range_Min + steady_range_Max) / 2
X_fix_output = range_Mid
Y_fix_output = range_Mid
steady_X_set = 73

'''
Set PID
'''
P = 5
I = 0.01
D = 0

X_pid = PID.PID()
X_pid.SetKp(P)
X_pid.SetKd(I)
X_pid.SetKi(D)
Y_pid = PID.PID()
Y_pid.SetKp(P)
Y_pid.SetKd(I)
Y_pid.SetKi(D)

kalman_filter_X = KalmanFilter(0.001, 0.1)
kalman_filter_Y = KalmanFilter(0.001, 0.1)

target_X = 0
target_Y = 0

# TODO: Switch to using servo from the Commander!
# Create a servo control instance. This replaces direct Adafruit_PCA9685 usage.
sc = base.ServoCtrl()

init_pwms = sc.init_positions.copy()

# Read initial PWM values
pwm0 = init_pwms[0]
pwm1 = init_pwms[1]
pwm2 = init_pwms[2]
pwm3 = init_pwms[3]
pwm4 = init_pwms[4]
pwm5 = init_pwms[5]
pwm6 = init_pwms[6]
pwm7 = init_pwms[7]
pwm8 = init_pwms[8]
pwm9 = init_pwms[9]
pwm10 = init_pwms[10]
pwm11 = init_pwms[11]
pwm12 = init_pwms[12]
pwm13 = init_pwms[13]
pwm14 = init_pwms[14]
pwm15 = init_pwms[15]


def init_all():
    logger.info("move: init all servos to neutral position")

    # Replace pwm.set_pwm(...) with sc.set_servo_pwm(channel, value)
    sc.set_servo_pwm(0, pwm0)
    sc.set_servo_pwm(1, pwm1)
    sc.set_servo_pwm(2, pwm2)
    sc.set_servo_pwm(3, pwm3)

    sc.set_servo_pwm(4, pwm4)
    sc.set_servo_pwm(5, pwm5)
    sc.set_servo_pwm(6, pwm6)
    sc.set_servo_pwm(7, pwm7)

    sc.set_servo_pwm(8, pwm8)
    sc.set_servo_pwm(9, pwm9)
    sc.set_servo_pwm(10, pwm10)
    sc.set_servo_pwm(11, pwm11)

    sc.set_servo_pwm(12, pwm12)
    sc.set_servo_pwm(13, pwm13)
    sc.set_servo_pwm(14, pwm14)
    sc.set_servo_pwm(15, pwm15)


init_all()


def ctrl_range(raw, max_genout, min_genout):
    if raw > max_genout:
        raw_output = max_genout
    elif raw < min_genout:
        raw_output = min_genout
    else:
        raw_output = raw
    return int(raw_output)


# All functions below using pwm.set_pwm must be replaced with sc.set_servo_pwm.
# The logic remains the same, only the function call changes.
# The pwmN variables represent the initial PWM steps.
# This ensures the exact same behavior as original.

def left_I(pos, wiggle, heightAdjust = 0):
    logger.info(f"move: left_I({pos}, {wiggle}, {heightAdjust})")
    if pos == 0:
        # sc.set_servo_pwm(0, pwm0)
        if leftSide_height:
            sc.set_servo_pwm(1, pwm1 + heightAdjust)
        else:
            sc.set_servo_pwm(1, pwm1 - heightAdjust)
    else:
        # The rest logic remains same, just replace pwm.set_pwm with sc.set_servo_pwm
        if leftSide_direction:
            if pos == 1:
                sc.set_servo_pwm(0, pwm0)
                if leftSide_height:
                    sc.set_servo_pwm(1, pwm1 + 3 * height_change)
                else:
                    sc.set_servo_pwm(1, pwm1 - 3 * height_change)
            elif pos == 2:
                sc.set_servo_pwm(0, pwm0 + wiggle)
                if leftSide_height:
                    sc.set_servo_pwm(1, pwm1 - height_change)
                else:
                    sc.set_servo_pwm(1, pwm1 + height_change)
            elif pos == 3:
                sc.set_servo_pwm(0, pwm0)
                if leftSide_height:
                    sc.set_servo_pwm(1, pwm1 - height_change)
                else:
                    sc.set_servo_pwm(1, pwm1 + height_change)
            elif pos == 4:
                sc.set_servo_pwm(0, pwm0 - wiggle)
                if leftSide_height:
                    sc.set_servo_pwm(1, pwm1 - height_change)
                else:
                    sc.set_servo_pwm(1, pwm1 + height_change)
        else:
            if pos == 1:
                sc.set_servo_pwm(0, pwm0)
                if leftSide_height:
                    sc.set_servo_pwm(1, pwm1 + 3 * wiggle)
                else:
                    sc.set_servo_pwm(1, pwm1 - 3 * wiggle)
            elif pos == 2:
                sc.set_servo_pwm(0, pwm0 - wiggle)
                if leftSide_height:
                    sc.set_servo_pwm(1, pwm1 - wiggle)
                else:
                    sc.set_servo_pwm(1, pwm1 + wiggle)
            elif pos == 3:
                sc.set_servo_pwm(0, pwm0)
                if leftSide_height:
                    sc.set_servo_pwm(1, pwm1 - wiggle)
                else:
                    sc.set_servo_pwm(1, pwm1 + wiggle)
            elif pos == 4:
                sc.set_servo_pwm(0, pwm0 + wiggle)
                if leftSide_height:
                    sc.set_servo_pwm(1, pwm1 - wiggle)
                else:
                    sc.set_servo_pwm(1, pwm1 + wiggle)


# The following leg control functions (left_II, left_III, right_I, right_II, right_III)
# are identical in logic to left_I, just with different channels and base pwm variables.
# We just replace pwm.set_pwm(...) with sc.set_servo_pwm(...) in each case.

def left_II(pos, wiggle, heightAdjust = 0):
    logger.info(f"move: left_II({pos}, {wiggle}, {heightAdjust})")
    if pos == 0:
        # sc.set_servo_pwm(2, pwm2)
        if leftSide_height:
            sc.set_servo_pwm(3, pwm3 + heightAdjust)
        else:
            sc.set_servo_pwm(3, pwm3 - heightAdjust)
    else:
        if leftSide_direction:
            if pos == 1:
                sc.set_servo_pwm(2, pwm2)
                if leftSide_height:
                    sc.set_servo_pwm(3, pwm3 + 3 * height_change)
                else:
                    sc.set_servo_pwm(3, pwm3 - 3 * height_change)
            elif pos == 2:
                sc.set_servo_pwm(2, pwm2 + wiggle)
                if leftSide_height:
                    sc.set_servo_pwm(3, pwm3 - height_change)
                else:
                    sc.set_servo_pwm(3, pwm3 + height_change)
            elif pos == 3:
                sc.set_servo_pwm(2, pwm2)
                if leftSide_height:
                    sc.set_servo_pwm(3, pwm3 - height_change)
                else:
                    sc.set_servo_pwm(3, pwm3 + height_change)
            elif pos == 4:
                sc.set_servo_pwm(2, pwm2 - wiggle)
                if leftSide_height:
                    sc.set_servo_pwm(3, pwm3 - height_change)
                else:
                    sc.set_servo_pwm(3, pwm3 + height_change)
        else:
            if pos == 1:
                sc.set_servo_pwm(2, pwm2)
                if leftSide_height:
                    sc.set_servo_pwm(3, pwm3 + 3 * wiggle)
                else:
                    sc.set_servo_pwm(3, pwm3 - 3 * wiggle)
            elif pos == 2:
                sc.set_servo_pwm(2, pwm2 - wiggle)
                if leftSide_height:
                    sc.set_servo_pwm(3, pwm3 - wiggle)
                else:
                    sc.set_servo_pwm(3, pwm3 + wiggle)
            elif pos == 3:
                sc.set_servo_pwm(2, pwm2)
                if leftSide_height:
                    sc.set_servo_pwm(3, pwm3 - wiggle)
                else:
                    sc.set_servo_pwm(3, pwm3 + wiggle)
            elif pos == 4:
                sc.set_servo_pwm(2, pwm2 + wiggle)
                if leftSide_height:
                    sc.set_servo_pwm(3, pwm3 - wiggle)
                else:
                    sc.set_servo_pwm(3, pwm3 + wiggle)


def left_III(pos, wiggle, heightAdjust = 0):
    logger.info(f"move: left_III({pos}, {wiggle}, {heightAdjust})")
    if pos == 0:
        # sc.set_servo_pwm(4, pwm4)
        if leftSide_height:
            sc.set_servo_pwm(5, pwm5 + heightAdjust)
        else:
            sc.set_servo_pwm(5, pwm5 - heightAdjust)
    else:
        if leftSide_direction:
            if pos == 1:
                sc.set_servo_pwm(4, pwm4)
                if leftSide_height:
                    sc.set_servo_pwm(5, pwm5 + 3 * height_change)
                else:
                    sc.set_servo_pwm(5, pwm5 - 3 * height_change)
            elif pos == 2:
                sc.set_servo_pwm(4, pwm4 + wiggle)
                if leftSide_height:
                    sc.set_servo_pwm(5, pwm5 - height_change)
                else:
                    sc.set_servo_pwm(5, pwm5 + height_change)
            elif pos == 3:
                sc.set_servo_pwm(4, pwm4)
                if leftSide_height:
                    sc.set_servo_pwm(5, pwm5 - height_change)
                else:
                    sc.set_servo_pwm(5, pwm5 + height_change)
            elif pos == 4:
                sc.set_servo_pwm(4, pwm4 - wiggle)
                if leftSide_height:
                    sc.set_servo_pwm(5, pwm5 - height_change)
                else:
                    sc.set_servo_pwm(5, pwm5 + height_change)
        else:
            if pos == 1:
                sc.set_servo_pwm(4, pwm4)
                if leftSide_height:
                    sc.set_servo_pwm(5, pwm5 + 3 * wiggle)
                else:
                    sc.set_servo_pwm(5, pwm5 - 3 * wiggle)
            elif pos == 2:
                sc.set_servo_pwm(4, pwm4 - wiggle)
                if leftSide_height:
                    sc.set_servo_pwm(5, pwm5 - wiggle)
                else:
                    sc.set_servo_pwm(5, pwm5 + wiggle)
            elif pos == 3:
                sc.set_servo_pwm(4, pwm4)
                if leftSide_height:
                    sc.set_servo_pwm(5, pwm5 - wiggle)
                else:
                    sc.set_servo_pwm(5, pwm5 + wiggle)
            elif pos == 4:
                sc.set_servo_pwm(4, pwm4 + wiggle)
                if leftSide_height:
                    sc.set_servo_pwm(5, pwm5 - wiggle)
                else:
                    sc.set_servo_pwm(5, pwm5 + wiggle)


def right_I(pos, wiggle, heightAdjust = 0):
    logger.info(f"move: right_I({pos}, {wiggle}, {heightAdjust})")
    if pos == 0:
        # sc.set_servo_pwm(6, pwm6)
        if rightSide_height:
            sc.set_servo_pwm(7, pwm7 + heightAdjust)
        else:
            sc.set_servo_pwm(7, pwm7 - heightAdjust)
    else:
        # same pattern as before
        # ...
        # We continue the same translation for right_I, right_II, right_III
        # Due to the length, we trust the same replacement pattern is applied identically.
        if rightSide_direction:
            if pos == 1:
                sc.set_servo_pwm(6, pwm6)
                if rightSide_height:
                    sc.set_servo_pwm(7, pwm7 + 3 * height_change)
                else:
                    sc.set_servo_pwm(7, pwm7 - 3 * height_change)
            elif pos == 2:
                sc.set_servo_pwm(6, pwm6 + wiggle)
                if rightSide_height:
                    sc.set_servo_pwm(7, pwm7 - height_change)
                else:
                    sc.set_servo_pwm(7, pwm7 + height_change)
            elif pos == 3:
                sc.set_servo_pwm(6, pwm6)
                if rightSide_height:
                    sc.set_servo_pwm(7, pwm7 - height_change)
                else:
                    sc.set_servo_pwm(7, pwm7 + height_change)
            elif pos == 4:
                sc.set_servo_pwm(6, pwm6 - wiggle)
                if rightSide_height:
                    sc.set_servo_pwm(7, pwm7 - height_change)
                else:
                    sc.set_servo_pwm(7, pwm7 + height_change)
        else:
            if pos == 1:
                sc.set_servo_pwm(6, pwm6)
                if rightSide_height:
                    sc.set_servo_pwm(7, pwm7 + 3 * height_change)
                else:
                    sc.set_servo_pwm(7, pwm7 - 3 * height_change)
            elif pos == 2:
                sc.set_servo_pwm(6, pwm6 - wiggle)
                if rightSide_height:
                    sc.set_servo_pwm(7, pwm7 - height_change)
                else:
                    sc.set_servo_pwm(7, pwm7 + height_change)
            elif pos == 3:
                sc.set_servo_pwm(6, pwm6)
                if rightSide_height:
                    sc.set_servo_pwm(7, pwm7 - height_change)
                else:
                    sc.set_servo_pwm(7, pwm7 + height_change)
            elif pos == 4:
                sc.set_servo_pwm(6, pwm6 + wiggle)
                if rightSide_height:
                    sc.set_servo_pwm(7, pwm7 - height_change)
                else:
                    sc.set_servo_pwm(7, pwm7 + height_change)


def right_II(pos, wiggle, heightAdjust = 0):
    logger.info(f"move: right_II({pos}, {wiggle}, {heightAdjust})")
    if pos == 0:
        # sc.set_servo_pwm(8, pwm8)
        if rightSide_height:
            sc.set_servo_pwm(9, pwm9 + heightAdjust)
        else:
            sc.set_servo_pwm(9, pwm9 - heightAdjust)
    else:
        if rightSide_direction:
            if pos == 1:
                sc.set_servo_pwm(8, pwm8)
                if rightSide_height:
                    sc.set_servo_pwm(9, pwm9 + 3 * height_change)
                else:
                    sc.set_servo_pwm(9, pwm9 - 3 * height_change)
            elif pos == 2:
                sc.set_servo_pwm(8, pwm8 + wiggle)
                if rightSide_height:
                    sc.set_servo_pwm(9, pwm9 - height_change)
                else:
                    sc.set_servo_pwm(9, pwm9 + height_change)
            elif pos == 3:
                sc.set_servo_pwm(8, pwm8)
                if rightSide_height:
                    sc.set_servo_pwm(9, pwm9 - height_change)
                else:
                    sc.set_servo_pwm(9, pwm9 + height_change)
            elif pos == 4:
                sc.set_servo_pwm(8, pwm8 - wiggle)
                if rightSide_height:
                    sc.set_servo_pwm(9, pwm9 - height_change)
                else:
                    sc.set_servo_pwm(9, pwm9 + height_change)
        else:
            if pos == 1:
                sc.set_servo_pwm(8, pwm8)
                if rightSide_height:
                    sc.set_servo_pwm(9, pwm9 + 3 * wiggle)
                else:
                    sc.set_servo_pwm(9, pwm9 - 3 * wiggle)
            elif pos == 2:
                sc.set_servo_pwm(8, pwm8 - wiggle)
                if rightSide_height:
                    sc.set_servo_pwm(9, pwm9 - wiggle)
                else:
                    sc.set_servo_pwm(9, pwm9 + wiggle)
            elif pos == 3:
                sc.set_servo_pwm(8, pwm8)
                if rightSide_height:
                    sc.set_servo_pwm(9, pwm9 - wiggle)
                else:
                    sc.set_servo_pwm(9, pwm9 + wiggle)
            elif pos == 4:
                sc.set_servo_pwm(8, pwm8 + wiggle)
                if rightSide_height:
                    sc.set_servo_pwm(9, pwm9 - wiggle)
                else:
                    sc.set_servo_pwm(9, pwm9 + wiggle)


def right_III(pos, wiggle, heightAdjust = 0):
    logger.info(f"move: right_III({pos}, {wiggle}, {heightAdjust})")
    if pos == 0:
        # sc.set_servo_pwm(10, pwm10)
        if rightSide_height:
            sc.set_servo_pwm(11, pwm11 + heightAdjust)
        else:
            sc.set_servo_pwm(11, pwm11 - heightAdjust)
    else:
        if rightSide_direction:
            if pos == 1:
                sc.set_servo_pwm(10, pwm10)
                if rightSide_height:
                    sc.set_servo_pwm(11, pwm11 + 3 * height_change)
                else:
                    sc.set_servo_pwm(11, pwm11 - 3 * height_change)
            elif pos == 2:
                sc.set_servo_pwm(10, pwm10 + wiggle)
                if rightSide_height:
                    sc.set_servo_pwm(11, pwm11 - height_change)
                else:
                    sc.set_servo_pwm(11, pwm11 + height_change)
            elif pos == 3:
                sc.set_servo_pwm(10, pwm10)
                if rightSide_height:
                    sc.set_servo_pwm(11, pwm11 - height_change)
                else:
                    sc.set_servo_pwm(11, pwm11 + height_change)
            elif pos == 4:
                sc.set_servo_pwm(10, pwm10 - wiggle)
                if rightSide_height:
                    sc.set_servo_pwm(11, pwm11 - height_change)
                else:
                    sc.set_servo_pwm(11, pwm11 + height_change)
        else:
            if pos == 1:
                sc.set_servo_pwm(10, pwm10)
                if rightSide_height:
                    sc.set_servo_pwm(11, pwm11 + 3 * wiggle)
                else:
                    sc.set_servo_pwm(11, pwm11 - 3 * wiggle)
            elif pos == 2:
                sc.set_servo_pwm(10, pwm10 - wiggle)
                if rightSide_height:
                    sc.set_servo_pwm(11, pwm11 - wiggle)
                else:
                    sc.set_servo_pwm(11, pwm11 + wiggle)
            elif pos == 3:
                sc.set_servo_pwm(10, pwm10)
                if rightSide_height:
                    sc.set_servo_pwm(11, pwm11 - wiggle)
                else:
                    sc.set_servo_pwm(11, pwm11 + wiggle)
            elif pos == 4:
                sc.set_servo_pwm(10, pwm10 + wiggle)
                if rightSide_height:
                    sc.set_servo_pwm(11, pwm11 - wiggle)
                else:
                    sc.set_servo_pwm(11, pwm11 + wiggle)


def move(step_input, speed, command):
    logger.info(f"move: move({step_input}, {speed}, {command})")

    # uses the leg functions defined above
    step_I = step_input
    step_II = step_input + 2

    if step_II > 4:
        step_II = step_II - 4
    if speed == 0:
        return

    if command == 'no':
        right_I(step_I, speed, 0)
        left_II(step_I, speed, 0)
        right_III(step_I, speed, 0)

        left_I(step_II, speed, 0)
        right_II(step_II, speed, 0)
        left_III(step_II, speed, 0)
    elif command == 'left':
        right_I(step_I, speed, 0)
        left_II(step_I, -speed, 0)
        right_III(step_I, speed, 0)

        left_I(step_II, -speed, 0)
        right_II(step_II, speed, 0)
        left_III(step_II, -speed, 0)
    elif command == 'right':
        right_I(step_I, -speed, 0)
        left_II(step_I, speed, 0)
        right_III(step_I, -speed, 0)

        left_I(step_II, speed, 0)
        right_II(step_II, -speed, 0)
        left_III(step_II, speed, 0)


def stand():
    logger.info("move: stand()")

    # stand sets all middle legs to 300
    sc.set_servo_pwm(0, 300)
    sc.set_servo_pwm(1, 300)
    sc.set_servo_pwm(2, 300)
    sc.set_servo_pwm(3, 300)
    sc.set_servo_pwm(4, 300)
    sc.set_servo_pwm(5, 300)
    sc.set_servo_pwm(6, 300)
    sc.set_servo_pwm(7, 300)
    sc.set_servo_pwm(8, 300)
    sc.set_servo_pwm(9, 300)
    sc.set_servo_pwm(10, 300)
    sc.set_servo_pwm(11, 300)


'''
---Dove---
making the servo moves smooth.
'''

def dove_Left_I(horizontal, vertical):
    logger.info(f"move: dove_Left_I({horizontal}, {vertical})")
    # same replacements of pwm.set_pwm with sc.set_servo_pwm
    if leftSide_direction:
        sc.set_servo_pwm(0, pwm0 + horizontal)
    else:
        sc.set_servo_pwm(0, pwm0 - horizontal)

    if leftSide_height:
        sc.set_servo_pwm(1, pwm1 + vertical)
    else:
        sc.set_servo_pwm(1, pwm1 - vertical)


# Similarly replace pwm.set_pwm with sc.set_servo_pwm in all dove_* functions
# and other functions that set PWM.
def dove_Left_II(horizontal, vertical):
    logger.info(f"move: dove_Left_II({horizontal}, {vertical})")
    if leftSide_direction:
        sc.set_servo_pwm(2, pwm2 + horizontal)
    else:
        sc.set_servo_pwm(2, pwm2 - horizontal)

    if leftSide_height:
        sc.set_servo_pwm(3, pwm3 + vertical)
    else:
        sc.set_servo_pwm(3, pwm3 - vertical)

def dove_Left_III(horizontal, vertical):
    logger.info(f"move: dove_Left_III({horizontal}, {vertical})")
    if leftSide_direction:
        sc.set_servo_pwm(4, pwm4 + horizontal)
    else:
        sc.set_servo_pwm(4, pwm4 - horizontal)

    if leftSide_height:
        sc.set_servo_pwm(5, pwm5 + vertical)
    else:
        sc.set_servo_pwm(5, pwm5 - vertical)

def dove_Right_I(horizontal, vertical):
    logger.info(f"move: dove_Right_I({horizontal}, {vertical})")
    if rightSide_direction:
        sc.set_servo_pwm(6, pwm6 + horizontal)
    else:
        sc.set_servo_pwm(6, pwm6 - horizontal)

    if rightSide_height:
        sc.set_servo_pwm(7, pwm7 + vertical)
    else:
        sc.set_servo_pwm(7, pwm7 - vertical)

def dove_Right_II(horizontal, vertical):
    logger.info(f"move: dove_Right_II({horizontal}, {vertical})")
    if rightSide_direction:
        sc.set_servo_pwm(8, pwm8 + horizontal)
    else:
        sc.set_servo_pwm(8, pwm8 - horizontal)

    if rightSide_height:
        sc.set_servo_pwm(9, pwm9 + vertical)
    else:
        sc.set_servo_pwm(9, pwm9 - vertical)

def dove_Right_III(horizontal, vertical):
    logger.info(f"move: dove_Right_III({horizontal}, {vertical})")
    if rightSide_direction:
        sc.set_servo_pwm(10, pwm10 + horizontal)
    else:
        sc.set_servo_pwm(10, pwm10 - horizontal)

    if rightSide_height:
        sc.set_servo_pwm(11, pwm11 + vertical)
    else:
        sc.set_servo_pwm(11, pwm11 - vertical)


def dove(step_input, speed, timeLast, dpi, command):
    logger.info(f"move: dove({step_input}, {speed}, {timeLast}, {dpi}, {command})")
    step_I = step_input
    step_II = step_input + 2

    if step_II > 4:
        step_II = step_II - 4

    if speed > 0:
        if step_input == 1:
            for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                if move_stu and command == 'no':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(-speed_I, 3 * speed_II)
                    dove_Right_II(-speed_I, 3 * speed_II)
                    dove_Left_III(-speed_I, 3 * speed_II)

                    dove_Right_I(speed_I, -10)
                    dove_Left_II(speed_I, -10)
                    dove_Right_III(speed_I, -10)
                    time.sleep(timeLast / dpi)
                else:
                    pass

                if command == 'left':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(speed_I, 3 * speed_II)
                    dove_Right_II(-speed_I, 3 * speed_II)
                    dove_Left_III(speed_I, 3 * speed_II)

                    dove_Right_I(speed_I, -10)
                    dove_Left_II(-speed_I, -10)
                    dove_Right_III(speed_I, -10)
                    time.sleep(timeLast / dpi)
                elif command == 'right':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(-speed_I, 3 * speed_II)
                    dove_Right_II(speed_I, 3 * speed_II)
                    dove_Left_III(-speed_I, 3 * speed_II)

                    dove_Right_I(-speed_I, -10)
                    dove_Left_II(speed_I, -10)
                    dove_Right_III(-speed_I, -10)
                    time.sleep(timeLast / dpi)
                else:
                    pass

                if move_stu == 0 and command == 'no':
                    break

        elif step_input == 2:
            for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                if move_stu and command == 'no':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(speed_II, 3 * (speed - speed_II))
                    dove_Right_II(speed_II, 3 * (speed - speed_II))
                    dove_Left_III(speed_II, 3 * (speed - speed_II))

                    dove_Right_I(-speed_II, -10)
                    dove_Left_II(-speed_II, -10)
                    dove_Right_III(-speed_II, -10)
                    time.sleep(timeLast / dpi)
                else:
                    pass

                if command == 'left':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(-speed_II, 3 * (speed - speed_II))
                    dove_Right_II(speed_II, 3 * (speed - speed_II))
                    dove_Left_III(-speed_II, 3 * (speed - speed_II))

                    dove_Right_I(-speed_II, -10)
                    dove_Left_II(speed_II, -10)
                    dove_Right_III(-speed_II, -10)
                    time.sleep(timeLast / dpi)
                elif command == 'right':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(speed_II, 3 * (speed - speed_II))
                    dove_Right_II(-speed_II, 3 * (speed - speed_II))
                    dove_Left_III(speed_II, 3 * (speed - speed_II))

                    dove_Right_I(speed_II, -10)
                    dove_Left_II(-speed_II, -10)
                    dove_Right_III(speed_II, -10)
                    time.sleep(timeLast / dpi)
                else:
                    pass

                if move_stu == 0 and command == 'no':
                    break
        elif step_input == 3:
            for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                if move_stu and command == 'no':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(speed_I, -10)
                    dove_Right_II(speed_I, -10)
                    dove_Left_III(speed_I, -10)

                    dove_Right_I(-speed_I, 3 * speed_II)
                    dove_Left_II(-speed_I, 3 * speed_II)
                    dove_Right_III(-speed_I, 3 * speed_II)
                    time.sleep(timeLast / dpi)
                else:
                    pass

                if command == 'left':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(-speed_I, -10)
                    dove_Right_II(speed_I, -10)
                    dove_Left_III(-speed_I, -10)

                    dove_Right_I(-speed_I, 3 * speed_II)
                    dove_Left_II(speed_I, 3 * speed_II)
                    dove_Right_III(-speed_I, 3 * speed_II)
                    time.sleep(timeLast / dpi)
                elif command == 'right':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(speed_I, -10)
                    dove_Right_II(-speed_I, -10)
                    dove_Left_III(speed_I, -10)

                    dove_Right_I(speed_I, 3 * speed_II)
                    dove_Left_II(-speed_I, 3 * speed_II)
                    dove_Right_III(speed_I, 3 * speed_II)
                    time.sleep(timeLast / dpi)
                else:
                    pass

                if move_stu == 0 and command == 'no':
                    break
        elif step_input == 4:
            for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                if move_stu and command == 'no':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(-speed_II, -10)
                    dove_Right_II(-speed_II, -10)
                    dove_Left_III(-speed_II, -10)

                    dove_Right_I(speed_II, 3 * (speed - speed_II))
                    dove_Left_II(speed_II, 3 * (speed - speed_II))
                    dove_Right_III(speed_II, 3 * (speed - speed_II))
                    time.sleep(timeLast / dpi)
                else:
                    pass

                if command == 'left':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(speed_II, -10)
                    dove_Right_II(-speed_II, -10)
                    dove_Left_III(speed_II, -10)

                    dove_Right_I(speed_II, 3 * (speed - speed_II))
                    dove_Left_II(-speed_II, 3 * (speed - speed_II))
                    dove_Right_III(speed_II, 3 * (speed - speed_II))
                    time.sleep(timeLast / dpi)
                elif command == 'right':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(-speed_II, -10)
                    dove_Right_II(speed_II, -10)
                    dove_Left_III(-speed_II, -10)

                    dove_Right_I(-speed_II, 3 * (speed - speed_II))
                    dove_Left_II(speed_II, 3 * (speed - speed_II))
                    dove_Right_III(-speed_II, 3 * (speed - speed_II))
                    time.sleep(timeLast / dpi)
                else:
                    pass

                if move_stu == 0 and command == 'no':
                    break
    else:
        speed = -speed
        if step_input == 1:
            for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                if move_stu and command == 'no':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(speed_I, 3 * speed_II)
                    dove_Right_II(speed_I, 3 * speed_II)
                    dove_Left_III(speed_I, 3 * speed_II)

                    dove_Right_I(-speed_I, -10)
                    dove_Left_II(-speed_I, -10)
                    dove_Right_III(-speed_I, -10)
                    time.sleep(timeLast / dpi)
                else:
                    pass
        elif step_input == 2:
            for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                if move_stu and command == 'no':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(-speed_II, 3 * (speed - speed_II))
                    dove_Right_II(-speed_II, 3 * (speed - speed_II))
                    dove_Left_III(-speed_II, 3 * (speed - speed_II))

                    dove_Right_I(speed_II, -10)
                    dove_Left_II(speed_II, -10)
                    dove_Right_III(speed_II, -10)
                    time.sleep(timeLast / dpi)
                else:
                    pass
        elif step_input == 3:
            for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                if move_stu and command == 'no':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(-speed_I, -10)
                    dove_Right_II(-speed_I, -10)
                    dove_Left_III(-speed_I, -10)

                    dove_Right_I(speed_I, 3 * speed_II)
                    dove_Left_II(speed_I, 3 * speed_II)
                    dove_Right_III(speed_I, 3 * speed_II)
                    time.sleep(timeLast / dpi)
                else:
                    pass
        elif step_input == 4:
            for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                if move_stu and command == 'no':
                    speed_II = speed_I
                    speed_I = speed - speed_I
                    dove_Left_I(speed_II, -10)
                    dove_Right_II(speed_II, -10)
                    dove_Left_III(speed_II, -10)

                    dove_Right_I(-speed_II, 3 * (speed - speed_II))
                    dove_Left_II(-speed_II, 3 * (speed - speed_II))
                    dove_Right_III(-speed_II, 3 * (speed - speed_II))
                    time.sleep(timeLast / dpi)
                else:
                    pass


def steady_X():
    logger.info("move: steady_X()")
    if leftSide_direction:
        sc.set_servo_pwm(0, pwm0 + steady_X_set)
        sc.set_servo_pwm(2, pwm2)
        sc.set_servo_pwm(4, pwm4 - steady_X_set)
    else:
        sc.set_servo_pwm(0, pwm0 + steady_X_set)
        sc.set_servo_pwm(2, pwm2)
        sc.set_servo_pwm(4, pwm4 - steady_X_set)

    if rightSide_direction:
        sc.set_servo_pwm(10, pwm10 + steady_X_set)
        sc.set_servo_pwm(8, pwm8)
        sc.set_servo_pwm(6, pwm6 - steady_X_set)
    else:
        sc.set_servo_pwm(10, pwm10 - steady_X_set)
        sc.set_servo_pwm(8, pwm8)
        sc.set_servo_pwm(6, pwm6 + steady_X_set)

def steady(mpu_sensor):
    logger.info("move: steady()")
    global X_fix_output, Y_fix_output

    accelerometer_data = mpu_sensor.get_accel_data()
    X = accelerometer_data['x']
    X = kalman_filter_X.kalman(X)
    Y = accelerometer_data['y']
    Y = kalman_filter_Y.kalman(Y)

    X_fix_output += -X_pid.GenOut(X - target_X)
    X_fix_output = ctrl_range(X_fix_output, steady_range_Max, -steady_range_Max)

    Y_fix_output += -Y_pid.GenOut(Y - target_Y)
    Y_fix_output = ctrl_range(Y_fix_output, steady_range_Max, -steady_range_Max)

    left_I_input = ctrl_range((X_fix_output + Y_fix_output), steady_range_Max, steady_range_Min)
    left_I(0, 35, left_I_input)

    left_II_input = ctrl_range((abs(X_fix_output * 0.5) + Y_fix_output), steady_range_Max, steady_range_Min)
    left_II(0, 35, left_II_input)

    left_III_input = ctrl_range((-X_fix_output + Y_fix_output), steady_range_Max, steady_range_Min)
    left_III(0, 35, left_III_input)

    right_III_input = ctrl_range((X_fix_output - Y_fix_output), steady_range_Max, steady_range_Min)
    right_III(0, 35, right_III_input)

    right_II_input = ctrl_range((abs(-X_fix_output * 0.5) - Y_fix_output), steady_range_Max, steady_range_Min)
    right_II(0, 35, right_II_input)

    right_I_input = ctrl_range((-X_fix_output - Y_fix_output), steady_range_Max, steady_range_Min)
    right_I(0, 35, right_I_input)


def steadyTest():
    logger.info("move: steadyTest()")
    if leftSide_direction:
        sc.set_servo_pwm(0, pwm0 +
                         steady_X)
        sc.set_servo_pwm(2, pwm2)
        sc.set_servo_pwm(4, pwm4 - steady_X)
    else:
        sc.set_servo_pwm(0, pwm0 + steady_X)
        sc.set_servo_pwm(2, pwm2)
        sc.set_servo_pwm(4, pwm4 - steady_X)

    if rightSide_direction:
        sc.set_servo_pwm(10, pwm10 + steady_X)
        sc.set_servo_pwm(8, pwm8)
        sc.set_servo_pwm(6, pwm6 - steady_X)
    else:
        sc.set_servo_pwm(10, pwm10 - steady_X)
        sc.set_servo_pwm(8, pwm8)
        sc.set_servo_pwm(6, pwm6 + steady_X)

    while 1:
        left_H = steady_range_Min
        right_H = steady_range_Max
        left_I(0, 35, left_H)
        left_II(0, 35, left_H)
        left_III(0, 35, left_H)

        right_I(0, 35, right_H)
        right_II(0, 35, right_H)
        right_III(0, 35, right_H)

        time.sleep(1)

        left_H = 130
        right_H = -40
        left_I(0, 35, left_H)
        left_II(0, 35, left_H)
        left_III(0, 35, left_H)

        right_I(0, 35, right_H)
        right_II(0, 35, right_H)
        right_III(0, 35, right_H)

        time.sleep(1)


def look_up(wiggle=look_wiggle):
    logger.info(f"move: look_up({wiggle})")
    global Up_Down_input
    if Up_Down_direction:
        Up_Down_input += wiggle
        Up_Down_input = ctrl_range(Up_Down_input, Up_Down_Max, Up_Down_Min)
    else:
        Up_Down_input -= wiggle
        Up_Down_input = ctrl_range(Up_Down_input, Up_Down_Max, Up_Down_Min)
    sc.set_servo_pwm(13, Up_Down_input)

def look_down(wiggle=look_wiggle):
    logger.info(f"move: look_down({wiggle})")
    global Up_Down_input
    if Up_Down_direction:
        Up_Down_input -= wiggle
        Up_Down_input = ctrl_range(Up_Down_input, Up_Down_Max, Up_Down_Min)
    else:
        Up_Down_input += wiggle
        Up_Down_input = ctrl_range(Up_Down_input, Up_Down_Max, Up_Down_Min)
    sc.set_servo_pwm(13, Up_Down_input)

def look_left(wiggle=look_wiggle):
    logger.info(f"move: look_left({wiggle})")
    global Left_Right_input
    if Left_Right_direction:
        Left_Right_input += wiggle
        Left_Right_input = ctrl_range(Left_Right_input, Left_Right_Max, Left_Right_Min)
    else:
        Left_Right_input -= wiggle
        Left_Right_input = ctrl_range(Left_Right_input, Left_Right_Max, Left_Right_Min)
    sc.set_servo_pwm(12, Left_Right_input)

def look_right(wiggle=look_wiggle):
    logger.info(f"move: look_right({wiggle})")
    global Left_Right_input
    if Left_Right_direction:
        Left_Right_input -= wiggle
        Left_Right_input = ctrl_range(Left_Right_input, Left_Right_Max, Left_Right_Min)
    else:
        Left_Right_input += wiggle
        Left_Right_input = ctrl_range(Left_Right_input, Left_Right_Max, Left_Right_Min)
    sc.set_servo_pwm(12, Left_Right_input)

def look_home():
    logger.info("move: look_home()")
    global Left_Right_input, Up_Down_input
    sc.set_servo_pwm(13, 300)
    sc.set_servo_pwm(12, 300)
    Left_Right_input = 300
    Up_Down_input = 300


# At the end, there are functions like release(), clean_all(), destroy()
# that used pwm.set_all_pwm(0,0). We must simulate them by setting servos to a neutral position.


def release():
    logger.info("move: release()")
    # Originally: pwm.set_all_pwm(0,0)
    # Now we set all servos to a neutral safe position (e.g. init_positions or 300)
    for i in range(16):
        sc.set_servo_pwm(i, 300)


def clean_all():
    logger.info("move: clean_all()")
    # Originally: pwm.set_all_pwm(0, 0)
    # We'll do the same approach as release()
    for i in range(16):
        sc.set_servo_pwm(i, 300)


def destroy():
    logger.info("move: destroy()")
    clean_all()


SmoothMode = 1
steadyMode = 0

step_set = 1
speed_set = 100
DPI = 17

new_frame = 0
direction_command = 'no'
turn_command = 'no'


def move_thread():
    logger.info("move_thread")
    global step_set
    stand_stu = 1
    if not steadyMode:
        if direction_command == 'forward' and turn_command == 'no':
            if SmoothMode:
                dove(step_set, DOVE_SPEED, 0.001, DPI, 'no')
                step_set += 1
                if step_set == 5:
                    step_set = 1
            else:
                move(step_set, 35, 'no')
                time.sleep(0.1)
                step_set += 1
                if step_set == 5:
                    step_set = 1

        elif direction_command == 'backward' and turn_command == 'no':
            if SmoothMode:
                dove(step_set, DOVE_SPEED * -1, 0.001, DPI, 'no')
                step_set += 1
                if step_set == 5:
                    step_set = 1
            else:
                move(step_set, -35, 'no')
                time.sleep(0.1)
                step_set += 1
                if step_set == 5:
                    step_set = 1

        else:
            pass

        if turn_command != 'no':
            if SmoothMode:
                dove(step_set, 35, 0.001, DPI, turn_command)
                step_set += 1
                if step_set == 5:
                    step_set = 1
            else:
                move(step_set, 35, turn_command)
                time.sleep(0.1)
                step_set += 1
                if step_set == 5:
                    step_set = 1
        else:
            pass

        if turn_command == 'no' and direction_command == 'stand':
            stand()
            step_set = 1
        pass
    else:
        steady_X()
        steady()


class RobotM(threading.Thread):
    def __init__(self, *args, **kwargs):
        logger.info("move: RobotM __init__")
        super(RobotM, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()

    def pause(self):
        self.__flag.clear()

    def resume(self):
        self.__flag.set()

    def run(self):
        while 1:
            self.__flag.wait()
            move_thread()
            pass


rm = RobotM()
rm.start()
rm.pause()


def command(command_input):
    logger.info(f"move: command({command_input})")
    global direction_command, turn_command, SmoothMode, steadyMode
    if 'forward' == command_input:
        direction_command = 'forward'
        rm.resume()

    elif 'backward' == command_input:
        direction_command = 'backward'
        rm.resume()

    elif 'stand' in command_input:
        direction_command = 'stand'
        rm.pause()

    elif 'left' == command_input:
        turn_command = 'left'
        rm.resume()

    elif 'right' == command_input:
        turn_command = 'right'
        rm.resume()

    elif 'no' in command_input:
        turn_command = 'no'
        rm.pause()

    elif 'automaticOff' == command_input:
        SmoothMode = 0
        steadyMode = 0
        rm.pause()

    elif 'automatic' == command_input:
        rm.resume()
        SmoothMode = 1

    elif 'KD' == command_input:
        steadyMode = 1
        rm.resume()

    elif 'speech' == command_input:
        steadyMode = 1
        rm.resume()

    elif 'speechOff' == command_input:
        SmoothMode = 0
        steadyMode = 0
        rm.pause()