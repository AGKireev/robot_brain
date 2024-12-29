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

# At module level
LEG_MAP = {
    'left_1':  {'horiz': 0, 'vert': 1, 'pwm_h': pwm0, 'pwm_v': pwm1, 'direction': leftSide_direction, 'height': leftSide_height},
    'left_2':  {'horiz': 2, 'vert': 3, 'pwm_h': pwm2, 'pwm_v': pwm3, 'direction': leftSide_direction, 'height': leftSide_height},
    'left_3':  {'horiz': 4, 'vert': 5, 'pwm_h': pwm4, 'pwm_v': pwm5, 'direction': leftSide_direction, 'height': leftSide_height},
    'right_1': {'horiz': 6, 'vert': 7, 'pwm_h': pwm6, 'pwm_v': pwm7, 'direction': rightSide_direction, 'height': rightSide_height},
    'right_2': {'horiz': 8, 'vert': 9, 'pwm_h': pwm8, 'pwm_v': pwm9, 'direction': rightSide_direction, 'height': rightSide_height},
    'right_3': {'horiz': 10, 'vert': 11, 'pwm_h': pwm10, 'pwm_v': pwm11, 'direction': rightSide_direction, 'height': rightSide_height}
}

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

def control_leg(leg_id, pos, wiggle, heightAdjust=0):
    """
    Universal leg control function that replaces individual left_* and right_* functions
    
    Args:
        leg_id: String identifying the leg ('left_1', 'left_2', 'left_3', 'right_1', 'right_2', 'right_3')
        pos: Position state (0-4)
        wiggle: Movement amount
        heightAdjust: Height adjustment value
    """
    logger.info(f"move: control_leg({leg_id}, {pos}, {wiggle}, {heightAdjust})")
    leg = LEG_MAP[leg_id]
    
    if pos == 0:
        if leg['height']:
            sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + heightAdjust)
        else:
            sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - heightAdjust)
    else:
        if leg['direction']:
            if pos == 1:
                sc.set_servo_pwm(leg['horiz'], leg['pwm_h'])
                if leg['height']:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + 3 * height_change)
                else:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - 3 * height_change)
            elif pos == 2:
                sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] + wiggle)
                if leg['height']:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - height_change)
                else:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + height_change)
            elif pos == 3:
                sc.set_servo_pwm(leg['horiz'], leg['pwm_h'])
                if leg['height']:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - height_change)
                else:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + height_change)
            elif pos == 4:
                sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] - wiggle)
                if leg['height']:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - height_change)
                else:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + height_change)
        else:
            if pos == 1:
                sc.set_servo_pwm(leg['horiz'], leg['pwm_h'])
                if leg['height']:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + 3 * wiggle)
                else:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - 3 * wiggle)
            elif pos == 2:
                sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] - wiggle)
                if leg['height']:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - wiggle)
                else:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + wiggle)
            elif pos == 3:
                sc.set_servo_pwm(leg['horiz'], leg['pwm_h'])
                if leg['height']:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - wiggle)
                else:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + wiggle)
            elif pos == 4:
                sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] + wiggle)
                if leg['height']:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - wiggle)
                else:
                    sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + wiggle)

# Replace old functions with wrappers that call control_leg
def left_I(pos, wiggle, heightAdjust=0):
    control_leg('left_1', pos, wiggle, heightAdjust)

def left_II(pos, wiggle, heightAdjust=0):
    control_leg('left_2', pos, wiggle, heightAdjust)

def left_III(pos, wiggle, heightAdjust=0):
    control_leg('left_3', pos, wiggle, heightAdjust)

def right_I(pos, wiggle, heightAdjust=0):
    control_leg('right_1', pos, wiggle, heightAdjust)

def right_II(pos, wiggle, heightAdjust=0):
    control_leg('right_2', pos, wiggle, heightAdjust)

def right_III(pos, wiggle, heightAdjust=0):
    control_leg('right_3', pos, wiggle, heightAdjust)


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

def dove_control_leg(leg_id, horizontal, vertical):
    """
    Universal dove control function that replaces individual dove_* functions
    
    Args:
        leg_id: String identifying the leg ('left_1', 'left_2', 'left_3', 'right_1', 'right_2', 'right_3')
        horizontal: Horizontal movement value
        vertical: Vertical movement value
    """
    logger.info(f"move: dove_control_leg({leg_id}, {horizontal}, {vertical})")
    leg = LEG_MAP[leg_id]
    
    if leg['direction']:
        sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] + horizontal)
    else:
        sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] - horizontal)

    if leg['height']:
        sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + vertical)
    else:
        sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - vertical)

# Replace old dove functions with wrappers
def dove_Left_I(horizontal, vertical):
    dove_control_leg('left_1', horizontal, vertical)

def dove_Left_II(horizontal, vertical):
    dove_control_leg('left_2', horizontal, vertical)

def dove_Left_III(horizontal, vertical):
    dove_control_leg('left_3', horizontal, vertical)

def dove_Right_I(horizontal, vertical):
    dove_control_leg('right_1', horizontal, vertical)

def dove_Right_II(horizontal, vertical):
    dove_control_leg('right_2', horizontal, vertical)

def dove_Right_III(horizontal, vertical):
    dove_control_leg('right_3', horizontal, vertical)


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