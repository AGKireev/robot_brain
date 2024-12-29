import time
import threading
import logging
from typing import Optional

from servo import base
from system.kalman_filter import KalmanFilter
import PID

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RobotMovement:
    def __init__(self, servo_ctrl: base.ServoCtrl):
        self.sc = servo_ctrl
        self.init_pwms = self.sc.init_positions.copy()
        
        # Configuration
        self.set_direction = 1
        self.leftSide_direction = 1 if self.set_direction else 0
        self.rightSide_direction = 0 if self.set_direction else 1
        self.leftSide_height = 0 if self.set_direction else 1
        self.rightSide_height = 1 if self.set_direction else 0
        
        # Movement configuration
        self.height_change = 30
        self.Up_Down_direction = 1 if self.set_direction else 0
        self.Left_Right_direction = 1 if self.set_direction else 0
        
        # Camera movement settings
        self.Left_Right_input = 300
        self.Up_Down_input = 300
        self.Left_Right_Max = 500
        self.Left_Right_Min = 100
        self.Up_Down_Max = 500
        self.Up_Down_Min = 270
        self.look_wiggle = 30
        
        # Movement state
        self.move_stu = 1
        self.DOVE_SPEED = 20
        
        # Steady mode configuration
        self.steady_range_Min = -40
        self.steady_range_Max = 130
        self.range_Mid = (self.steady_range_Min + self.steady_range_Max) / 2
        self.X_fix_output = self.range_Mid
        self.Y_fix_output = self.range_Mid
        self.steady_X_set = 73
        
        # PID Configuration
        self.P = 5
        self.I = 0.01
        self.D = 0
        
        self.X_pid = PID.PID()
        self.X_pid.SetKp(self.P)
        self.X_pid.SetKd(self.I)
        self.X_pid.SetKi(self.D)
        
        self.Y_pid = PID.PID()
        self.Y_pid.SetKp(self.P)
        self.Y_pid.SetKd(self.I)
        self.Y_pid.SetKi(self.D)
        
        self.kalman_filter_X = KalmanFilter(0.001, 0.1)
        self.kalman_filter_Y = KalmanFilter(0.001, 0.1)
        
        self.target_X = 0
        self.target_Y = 0
        
        # Movement thread state
        self.step_set = 1
        self.speed_set = 100
        self.DPI = 17
        self.direction_command = 'no'
        self.turn_command = 'no'
        self.SmoothMode = 1
        self.steadyMode = 0
        
        # Initialize leg mapping
        self._init_leg_map()
        
        # Initialize movement thread
        self.movement_thread = self._create_movement_thread()
        self.movement_thread.start()
        self.movement_thread.pause()

    def _init_leg_map(self):
        """Initialize the leg mapping configuration"""
        self.LEG_MAP = {
            'left_1':  {'horiz': 0, 'vert': 1, 'pwm_h': self.init_pwms[0], 'pwm_v': self.init_pwms[1], 
                       'direction': self.leftSide_direction, 'height': self.leftSide_height},
            'left_2':  {'horiz': 2, 'vert': 3, 'pwm_h': self.init_pwms[2], 'pwm_v': self.init_pwms[3], 
                       'direction': self.leftSide_direction, 'height': self.leftSide_height},
            'left_3':  {'horiz': 4, 'vert': 5, 'pwm_h': self.init_pwms[4], 'pwm_v': self.init_pwms[5], 
                       'direction': self.leftSide_direction, 'height': self.leftSide_height},
            'right_1': {'horiz': 6, 'vert': 7, 'pwm_h': self.init_pwms[6], 'pwm_v': self.init_pwms[7], 
                       'direction': self.rightSide_direction, 'height': self.rightSide_height},
            'right_2': {'horiz': 8, 'vert': 9, 'pwm_h': self.init_pwms[8], 'pwm_v': self.init_pwms[9], 
                       'direction': self.rightSide_direction, 'height': self.rightSide_height},
            'right_3': {'horiz': 10, 'vert': 11, 'pwm_h': self.init_pwms[10], 'pwm_v': self.init_pwms[11], 
                       'direction': self.rightSide_direction, 'height': self.rightSide_height}
        }

    def _create_movement_thread(self):
        """Create the movement control thread"""
        class MovementThread(threading.Thread):
            def __init__(self, robot_movement):
                super().__init__()
                self.robot = robot_movement
                self.__flag = threading.Event()
                self.__flag.clear()

            def pause(self):
                self.__flag.clear()

            def resume(self):
                self.__flag.set()

            def run(self):
                while True:
                    self.__flag.wait()
                    self.robot._move_thread()

        return MovementThread(self)

    def command(self, command_input: str):
        """Process movement commands"""
        logger.info(f"move: command({command_input})")
        
        if command_input == 'forward':
            self.direction_command = 'forward'
            self.movement_thread.resume()
        elif command_input == 'backward':
            self.direction_command = 'backward'
            self.movement_thread.resume()
        elif 'stand' in command_input:
            self.direction_command = 'stand'
            self.movement_thread.pause()
        elif command_input == 'left':
            self.turn_command = 'left'
            self.movement_thread.resume()
        elif command_input == 'right':
            self.turn_command = 'right'
            self.movement_thread.resume()
        elif 'no' in command_input:
            self.turn_command = 'no'
            self.movement_thread.pause()
        elif command_input == 'automaticOff':
            self.SmoothMode = 0
            self.steadyMode = 0
            self.movement_thread.pause()
        elif command_input == 'automatic':
            self.movement_thread.resume()
            self.SmoothMode = 1
        elif command_input == 'KD':
            self.steadyMode = 1
            self.movement_thread.resume()

    def control_leg(self, leg_id: str, pos: int, wiggle: int, heightAdjust: int = 0):
        """Control a single leg's movement"""
        logger.info(f"move: control_leg({leg_id}, {pos}, {wiggle}, {heightAdjust})")
        
        leg = self.LEG_MAP[leg_id]
        
        if pos == 0:
            if leg['height']:
                self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + heightAdjust)
            else:
                self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - heightAdjust)
        else:
            if leg['direction']:
                if pos == 1:
                    self.sc.set_servo_pwm(leg['horiz'], leg['pwm_h'])
                    if leg['height']:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + 3 * self.height_change)
                    else:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - 3 * self.height_change)
                elif pos == 2:
                    self.sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] + wiggle)
                    if leg['height']:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - self.height_change)
                    else:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + self.height_change)
                elif pos == 3:
                    self.sc.set_servo_pwm(leg['horiz'], leg['pwm_h'])
                    if leg['height']:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - self.height_change)
                    else:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + self.height_change)
                elif pos == 4:
                    self.sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] - wiggle)
                    if leg['height']:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - self.height_change)
                    else:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + self.height_change)
            else:
                if pos == 1:
                    self.sc.set_servo_pwm(leg['horiz'], leg['pwm_h'])
                    if leg['height']:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + 3 * wiggle)
                    else:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - 3 * wiggle)
                elif pos == 2:
                    self.sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] - wiggle)
                    if leg['height']:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - wiggle)
                    else:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + wiggle)
                elif pos == 3:
                    self.sc.set_servo_pwm(leg['horiz'], leg['pwm_h'])
                    if leg['height']:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - wiggle)
                    else:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + wiggle)
                elif pos == 4:
                    self.sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] + wiggle)
                    if leg['height']:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - wiggle)
                    else:
                        self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + wiggle)

    def dove_control_leg(self, leg_id: str, horizontal: int, vertical: int):
        """Control a single leg's dove movement"""
        logger.info(f"move: dove_control_leg({leg_id}, {horizontal}, {vertical})")
        
        leg = self.LEG_MAP[leg_id]
        
        if leg['direction']:
            self.sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] + horizontal)
        else:
            self.sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] - horizontal)

        if leg['height']:
            self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + vertical)
        else:
            self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - vertical)

    def move(self, step_input: int, speed: int, command: str):
        """Execute a movement sequence"""
        logger.info(f"move: move({step_input}, {speed}, {command})")

        step_I = step_input
        step_II = step_input + 2

        if step_II > 4:
            step_II = step_II - 4
        if speed == 0:
            return

        if command == 'no':
            self.control_leg('right_1', step_I, speed)
            self.control_leg('left_2', step_I, speed)
            self.control_leg('right_3', step_I, speed)

            self.control_leg('left_1', step_II, speed)
            self.control_leg('right_2', step_II, speed)
            self.control_leg('left_3', step_II, speed)
        elif command == 'left':
            self.control_leg('right_1', step_I, speed)
            self.control_leg('left_2', step_I, -speed)
            self.control_leg('right_3', step_I, speed)

            self.control_leg('left_1', step_II, -speed)
            self.control_leg('right_2', step_II, speed)
            self.control_leg('left_3', step_II, -speed)
        elif command == 'right':
            self.control_leg('right_1', step_I, -speed)
            self.control_leg('left_2', step_I, speed)
            self.control_leg('right_3', step_I, -speed)

            self.control_leg('left_1', step_II, speed)
            self.control_leg('right_2', step_II, -speed)
            self.control_leg('left_3', step_II, speed)

    def dove(self, step_input: int, speed: int, timeLast: float, dpi: int, command: str):
        """Execute a dove movement sequence"""
        logger.info(f"move: dove({step_input}, {speed}, {timeLast}, {dpi}, {command})")
        step_I = step_input
        step_II = step_input + 2

        if step_II > 4:
            step_II = step_II - 4

        if speed > 0:
            if step_input == 1:
                for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                    if self.move_stu and command == 'no':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_control_leg('left_1', -speed_I, 3 * speed_II)
                        self.dove_control_leg('right_2', -speed_I, 3 * speed_II)
                        self.dove_control_leg('left_3', -speed_I, 3 * speed_II)

                        self.dove_control_leg('right_1', speed_I, -10)
                        self.dove_control_leg('left_2', speed_I, -10)
                        self.dove_control_leg('right_3', speed_I, -10)
                        time.sleep(timeLast / dpi)
                    else:
                        pass

                    if command == 'left':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_control_leg('left_1', speed_I, 3 * speed_II)
                        self.dove_control_leg('right_2', -speed_I, 3 * speed_II)
                        self.dove_control_leg('left_3', speed_I, 3 * speed_II)

                        self.dove_control_leg('right_1', speed_I, -10)
                        self.dove_control_leg('left_2', -speed_I, -10)
                        self.dove_control_leg('right_3', speed_I, -10)
                        time.sleep(timeLast / dpi)
                    elif command == 'right':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_control_leg('left_1', -speed_I, 3 * speed_II)
                        self.dove_control_leg('right_2', speed_I, 3 * speed_II)
                        self.dove_control_leg('left_3', -speed_I, 3 * speed_II)

                        self.dove_control_leg('right_1', -speed_I, -10)
                        self.dove_control_leg('left_2', speed_I, -10)
                        self.dove_control_leg('right_3', -speed_I, -10)
                        time.sleep(timeLast / dpi)
                    else:
                        pass

                    if self.move_stu == 0 and command == 'no':
                        break

            elif step_input == 2:
                # Similar pattern for step_input == 2
                for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                    if self.move_stu and command == 'no':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_control_leg('left_1', speed_II, 3 * (speed - speed_II))
                        self.dove_control_leg('right_2', speed_II, 3 * (speed - speed_II))
                        self.dove_control_leg('left_3', speed_II, 3 * (speed - speed_II))

                        self.dove_control_leg('right_1', -speed_II, -10)
                        self.dove_control_leg('left_2', -speed_II, -10)
                        self.dove_control_leg('right_3', -speed_II, -10)
                        time.sleep(timeLast / dpi)
                    # ... continue with left/right commands similar to step_input == 1

            elif step_input == 3:
                # Similar pattern for step_input == 3
                for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                    if self.move_stu and command == 'no':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_control_leg('left_1', speed_I, -10)
                        self.dove_control_leg('right_2', speed_I, -10)
                        self.dove_control_leg('left_3', speed_I, -10)

                        self.dove_control_leg('right_1', -speed_I, 3 * speed_II)
                        self.dove_control_leg('left_2', -speed_I, 3 * speed_II)
                        self.dove_control_leg('right_3', -speed_I, 3 * speed_II)
                        time.sleep(timeLast / dpi)
                    # ... continue with left/right commands similar to step_input == 1

            elif step_input == 4:
                # Similar pattern for step_input == 4
                for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                    if self.move_stu and command == 'no':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_control_leg('left_1', -speed_II, -10)
                        self.dove_control_leg('right_2', -speed_II, -10)
                        self.dove_control_leg('left_3', -speed_II, -10)

                        self.dove_control_leg('right_1', speed_II, 3 * (speed - speed_II))
                        self.dove_control_leg('left_2', speed_II, 3 * (speed - speed_II))
                        self.dove_control_leg('right_3', speed_II, 3 * (speed - speed_II))
                        time.sleep(timeLast / dpi)
                    # ... continue with left/right commands similar to step_input == 1

    def _move_thread(self):
        """Internal movement thread function"""
        if not self.steadyMode:
            if self.direction_command == 'forward' and self.turn_command == 'no':
                if self.SmoothMode:
                    self.dove(self.step_set, self.DOVE_SPEED, 0.001, self.DPI, 'no')
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1
                else:
                    self.move(self.step_set, 35, 'no')
                    time.sleep(0.1)
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1

            elif self.direction_command == 'backward' and self.turn_command == 'no':
                if self.SmoothMode:
                    self.dove(self.step_set, self.DOVE_SPEED * -1, 0.001, self.DPI, 'no')
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1
                else:
                    self.move(self.step_set, -35, 'no')
                    time.sleep(0.1)
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1

            if self.turn_command != 'no':
                if self.SmoothMode:
                    self.dove(self.step_set, 35, 0.001, self.DPI, self.turn_command)
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1
                else:
                    self.move(self.step_set, 35, self.turn_command)
                    time.sleep(0.1)
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1

            if self.turn_command == 'no' and self.direction_command == 'stand':
                self.stand()
                self.step_set = 1
        else:
            self.steady_X()
            self.steady()

    def stand(self):
        """Put robot in standing position"""
        logger.info("move: stand()")
        for i in range(12):
            self.sc.set_servo_pwm(i, 300)

    def ctrl_range(self, raw: float, max_genout: float, min_genout: float) -> int:
        """Control the range of a value"""
        if raw > max_genout:
            raw_output = max_genout
        elif raw < min_genout:
            raw_output = min_genout
        else:
            raw_output = raw
        return int(raw_output)

    def steady(self, mpu_sensor):
        """Maintain steady position using MPU sensor"""
        logger.info("move: steady()")
        
        accelerometer_data = mpu_sensor.get_accel_data()
        X = accelerometer_data['x']
        X = self.kalman_filter_X.kalman(X)
        Y = accelerometer_data['y']
        Y = self.kalman_filter_Y.kalman(Y)

        self.X_fix_output += -self.X_pid.GenOut(X - self.target_X)
        self.X_fix_output = self.ctrl_range(self.X_fix_output, self.steady_range_Max, -self.steady_range_Max)

        self.Y_fix_output += -self.Y_pid.GenOut(Y - self.target_Y)
        self.Y_fix_output = self.ctrl_range(self.Y_fix_output, self.steady_range_Max, -self.steady_range_Max)

        # Apply steady adjustments to each leg
        for leg_id in self.LEG_MAP:
            if leg_id.startswith('left'):
                if leg_id == 'left_1':
                    input_val = self.ctrl_range((self.X_fix_output + self.Y_fix_output), 
                                             self.steady_range_Max, self.steady_range_Min)
                elif leg_id == 'left_2':
                    input_val = self.ctrl_range((abs(self.X_fix_output * 0.5) + self.Y_fix_output), 
                                             self.steady_range_Max, self.steady_range_Min)
                else:  # left_3
                    input_val = self.ctrl_range((-self.X_fix_output + self.Y_fix_output), 
                                             self.steady_range_Max, self.steady_range_Min)
            else:  # right legs
                if leg_id == 'right_3':
                    input_val = self.ctrl_range((self.X_fix_output - self.Y_fix_output), 
                                             self.steady_range_Max, self.steady_range_Min)
                elif leg_id == 'right_2':
                    input_val = self.ctrl_range((abs(-self.X_fix_output * 0.5) - self.Y_fix_output), 
                                             self.steady_range_Max, self.steady_range_Min)
                else:  # right_1
                    input_val = self.ctrl_range((-self.X_fix_output - self.Y_fix_output), 
                                             self.steady_range_Max, self.steady_range_Min)
            
            self.control_leg(leg_id, 0, 35, input_val)

    def release(self):
        """Release all servos to neutral position"""
        logger.info("move: release()")
        for i in range(16):
            self.sc.set_servo_pwm(i, 300)

    def clean_all(self):
        """Clean up all servo positions"""
        logger.info("move: clean_all()")
        self.release()

    def destroy(self):
        """Cleanup before destroying the instance"""
        logger.info("move: destroy()")
        self.clean_all()

    def look_up(self, wiggle: int = None):
        """Move camera up"""
        logger.info(f"move: look_up({wiggle if wiggle else self.look_wiggle})")
        if wiggle is None:
            wiggle = self.look_wiggle
            
        if self.Up_Down_direction:
            self.Up_Down_input += wiggle
            self.Up_Down_input = self.ctrl_range(self.Up_Down_input, self.Up_Down_Max, self.Up_Down_Min)
        else:
            self.Up_Down_input -= wiggle
            self.Up_Down_input = self.ctrl_range(self.Up_Down_input, self.Up_Down_Max, self.Up_Down_Min)
        self.sc.set_servo_pwm(13, self.Up_Down_input)

    def look_down(self, wiggle: int = None):
        """Move camera down"""
        logger.info(f"move: look_down({wiggle if wiggle else self.look_wiggle})")
        if wiggle is None:
            wiggle = self.look_wiggle
            
        if self.Up_Down_direction:
            self.Up_Down_input -= wiggle
            self.Up_Down_input = self.ctrl_range(self.Up_Down_input, self.Up_Down_Max, self.Up_Down_Min)
        else:
            self.Up_Down_input += wiggle
            self.Up_Down_input = self.ctrl_range(self.Up_Down_input, self.Up_Down_Max, self.Up_Down_Min)
        self.sc.set_servo_pwm(13, self.Up_Down_input)

    def look_left(self, wiggle: int = None):
        """Move camera left"""
        logger.info(f"move: look_left({wiggle if wiggle else self.look_wiggle})")
        if wiggle is None:
            wiggle = self.look_wiggle
            
        if self.Left_Right_direction:
            self.Left_Right_input += wiggle
            self.Left_Right_input = self.ctrl_range(self.Left_Right_input, self.Left_Right_Max, self.Left_Right_Min)
        else:
            self.Left_Right_input -= wiggle
            self.Left_Right_input = self.ctrl_range(self.Left_Right_input, self.Left_Right_Max, self.Left_Right_Min)
        self.sc.set_servo_pwm(12, self.Left_Right_input)

    def look_right(self, wiggle: int = None):
        """Move camera right"""
        logger.info(f"move: look_right({wiggle if wiggle else self.look_wiggle})")
        if wiggle is None:
            wiggle = self.look_wiggle
            
        if self.Left_Right_direction:
            self.Left_Right_input -= wiggle
            self.Left_Right_input = self.ctrl_range(self.Left_Right_input, self.Left_Right_Max, self.Left_Right_Min)
        else:
            self.Left_Right_input += wiggle
            self.Left_Right_input = self.ctrl_range(self.Left_Right_input, self.Left_Right_Max, self.Left_Right_Min)
        self.sc.set_servo_pwm(12, self.Left_Right_input)

    def look_home(self):
        """Reset camera to home position"""
        logger.info("move: look_home()")
        self.sc.set_servo_pwm(13, 300)
        self.sc.set_servo_pwm(12, 300)
        self.Left_Right_input = 300
        self.Up_Down_input = 300

    def steadyTest(self):
        """Test steady mode functionality"""
        logger.info("move: steadyTest()")
        if self.leftSide_direction:
            self.sc.set_servo_pwm(0, self.init_pwms[0] + self.steady_X_set)
            self.sc.set_servo_pwm(2, self.init_pwms[2])
            self.sc.set_servo_pwm(4, self.init_pwms[4] - self.steady_X_set)
        else:
            self.sc.set_servo_pwm(0, self.init_pwms[0] + self.steady_X_set)
            self.sc.set_servo_pwm(2, self.init_pwms[2])
            self.sc.set_servo_pwm(4, self.init_pwms[4] - self.steady_X_set)

        if self.rightSide_direction:
            self.sc.set_servo_pwm(10, self.init_pwms[10] + self.steady_X_set)
            self.sc.set_servo_pwm(8, self.init_pwms[8])
            self.sc.set_servo_pwm(6, self.init_pwms[6] - self.steady_X_set)
        else:
            self.sc.set_servo_pwm(10, self.init_pwms[10] - self.steady_X_set)
            self.sc.set_servo_pwm(8, self.init_pwms[8])
            self.sc.set_servo_pwm(6, self.init_pwms[6] + self.steady_X_set)

        while True:
            left_H = self.steady_range_Min
            right_H = self.steady_range_Max
            
            for leg_id in ['left_1', 'left_2', 'left_3']:
                self.control_leg(leg_id, 0, 35, left_H)
            for leg_id in ['right_1', 'right_2', 'right_3']:
                self.control_leg(leg_id, 0, 35, right_H)

            time.sleep(1)

            left_H = 130
            right_H = -40
            for leg_id in ['left_1', 'left_2', 'left_3']:
                self.control_leg(leg_id, 0, 35, left_H)
            for leg_id in ['right_1', 'right_2', 'right_3']:
                self.control_leg(leg_id, 0, 35, right_H)

            time.sleep(1)