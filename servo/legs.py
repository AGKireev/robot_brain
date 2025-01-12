import time
import threading
import logging
from typing import Dict, List, Optional, Union

from servo import base
from system.kalman_filter import KalmanFilter
import PID

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class LegsMovement:
    """Controls the movement of the hexapod robot's legs."""
    
    def __init__(self, servo_ctrl: base.ServoCtrl):
        """Initialize legs movement controller.
        
        Args:
            servo_ctrl: ServoCtrl instance configured for leg servos
        """
        self.sc = servo_ctrl
        self.init_pwms = self.sc.init_positions.copy()
        logger.info("Initializing LegsMovement with servo controller")
        
        # Get configuration
        self.config = base.servo_config['legs']
        logger.debug(f"Loaded leg configuration: {self.config}")
        
        # Movement parameters
        self.height_change = 30
        self.move_stu = 1
        self.DOVE_SPEED = 20
        
        # Movement state
        self.step_set = 1
        self.speed_set = 100
        self.DPI = 17
        self.direction_command = 'no'
        self.turn_command = 'no'
        self.SmoothMode = 1
        self.steadyMode = 0
        self.completing_movement = False  # New flag to track movement completion
        
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
        
        # Initialize PID controllers
        self.X_pid = PID.PID()
        self.X_pid.SetKp(self.P)
        self.X_pid.SetKd(self.I)
        self.X_pid.SetKi(self.D)
        
        self.Y_pid = PID.PID()
        self.Y_pid.SetKp(self.P)
        self.Y_pid.SetKd(self.I)
        self.Y_pid.SetKi(self.D)
        
        # Initialize Kalman filters
        self.kalman_filter_X = KalmanFilter(0.001, 0.1)
        self.kalman_filter_Y = KalmanFilter(0.001, 0.1)
        
        self.target_X = 0
        self.target_Y = 0
        
        # Initialize leg mapping
        self._init_leg_map()
        
        # Initialize movement thread
        self.movement_thread = self._create_movement_thread()
        self.movement_thread.start()
        self.movement_thread.pause()
        
        # MPU sensor will be set externally
        self.mpu_sensor = None
        
        logger.info("LegsMovement initialization complete")

    def _init_leg_map(self):
        """Initialize the leg mapping configuration.
        
        Leg scheme:
        left_I -<forward>-- right_III
        left_II ---<BODY>--- right_II
        left_III -<Backward>- right_I
        """
        self.LEG_MAP = {}
        
        # Map config names to our scheme
        config_to_scheme = {
            'front_left': 'left_I',
            'middle_left': 'left_II',
            'back_left': 'left_III',
            'front_right': 'right_III',
            'middle_right': 'right_II',
            'back_right': 'right_I'
        }
        
        # Build leg map from config
        for config_name, scheme_name in config_to_scheme.items():
            leg_config = self.config[config_name]
            channels = leg_config['channels']
            positions = leg_config['center_position']
            directions = leg_config['direction']
            
            self.LEG_MAP[scheme_name] = {
                'horiz': channels['horizontal'],
                'vert': channels['vertical'],
                'pwm_h': positions['horizontal'],
                'pwm_v': positions['vertical'],
                'direction': directions['horizontal'],  # Use horizontal direction as main direction
                'height': directions['vertical']  # Use vertical direction for height
            }
        
        logger.debug(f"Initialized leg mapping: {self.LEG_MAP}")

    def _create_movement_thread(self):
        """Create the movement control thread."""
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
                    # Add a small delay to prevent CPU spinning when idle
                    time.sleep(0.01)  # 10ms delay is short enough to not affect responsiveness

        return MovementThread(self)

    def command(self, command_input: str) -> None:
        """Process movement commands.
        
        Args:
            command_input: Command to execute (forward, backward, left, right, stand, no)
        """
        logger.info(f"Processing command: {command_input}")
        
        # Store previous state for logging
        prev_direction = self.direction_command
        prev_turn = self.turn_command
        
        if command_input == 'forward':
            self.direction_command = 'forward'
            self.turn_command = 'no'  # Clear any turn command
            self.completing_movement = False
            self.movement_thread.resume()
            logger.info(f"Starting forward movement (prev: direction={prev_direction}, turn={prev_turn})")
            
        elif command_input == 'backward':
            self.direction_command = 'backward'
            self.turn_command = 'no'  # Clear any turn command
            self.completing_movement = False
            self.movement_thread.resume()
            logger.info(f"Starting backward movement (prev: direction={prev_direction}, turn={prev_turn})")
            
        elif command_input == 'stand':
            self.direction_command = 'stand'
            self.turn_command = 'no'
            self.completing_movement = False
            self.movement_thread.pause()
            logger.info(f"Moving to stand position (prev: direction={prev_direction}, turn={prev_turn})")
            
        elif command_input == 'left':
            self.turn_command = 'left'
            self.direction_command = 'no'  # Clear any direction command
            self.completing_movement = False
            self.movement_thread.resume()
            logger.info(f"Starting left turn (prev: direction={prev_direction}, turn={prev_turn})")
            
        elif command_input == 'right':
            self.turn_command = 'right'
            self.direction_command = 'no'  # Clear any direction command
            self.completing_movement = False
            self.movement_thread.resume()
            logger.info(f"Starting right turn (prev: direction={prev_direction}, turn={prev_turn})")
            
        elif command_input == 'no':
            # If we were moving, set completing_movement flag
            if self.direction_command in ['forward', 'backward'] or self.turn_command in ['left', 'right']:
                self.completing_movement = True
                logger.info("Completing current movement cycle before stopping")
            else:
                self.completing_movement = False
                self.turn_command = 'no'
                self.direction_command = 'no'
                self.movement_thread.pause()
                logger.info(f"Stopping all movement (prev: direction={prev_direction}, turn={prev_turn})")
            
        elif command_input == 'automaticOff':
            self.SmoothMode = 0
            self.steadyMode = 0
            self.completing_movement = False
            self.movement_thread.pause()
            logger.info("Disabled automatic/smooth mode")
            
        elif command_input == 'automatic':
            self.movement_thread.resume()
            self.completing_movement = False
            self.SmoothMode = 1
            logger.info("Enabled automatic/smooth mode")
            
        elif command_input == 'KD':
            self.steadyMode = 1
            self.completing_movement = False
            self.movement_thread.resume()
            logger.info("Enabled steady mode with Kalman filter")
        
        else:
            logger.warning(f"Unknown command: {command_input}")

    def control_leg(self, leg_id: str, pos: int, wiggle: int, heightAdjust: int = 0) -> None:
        """Control a single leg's movement.
        
        Args:
            leg_id: ID of the leg to control
            pos: Position state (0-4)
            wiggle: Amount of wiggle movement
            heightAdjust: Height adjustment value
        """
        logger.debug(f"Controlling leg {leg_id} - pos: {pos}, wiggle: {wiggle}, height: {heightAdjust}")
        
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

    def move(self, step_input: int, speed: int, command: str) -> None:
        """Execute a movement sequence.
        
        Args:
            step_input: Current step in sequence (1-4)
            speed: Movement speed
            command: Movement command (no, left, right)
        """
        logger.debug(f"Moving - step: {step_input}, speed: {speed}, command: {command}")
        
        # Calculate second tripod step (offset by 2 from first tripod)
        step_II = step_input + 2
        if step_II > 4:
            step_II = step_II - 4
            
        if speed == 0:
            return

        # Height adjustment for lift phase
        lift_height = 50  # Increased lift height for better ground clearance

        if command == 'no':
            # First tripod: right_I (back right), left_II (middle left), right_III (front right)
            # In lift phase (step 1), increase height and move forward
            # In stance phase (step 2-4), maintain ground contact and move backward
            if step_input == 1:
                # Lift phase for first tripod
                self.control_leg('right_I', step_input, speed, lift_height)
                self.control_leg('left_II', step_input, speed, lift_height)
                self.control_leg('right_III', step_input, speed, lift_height)
                # Stance phase for second tripod
                self.control_leg('left_I', step_II, -speed)
                self.control_leg('right_II', step_II, -speed)
                self.control_leg('left_III', step_II, -speed)
            else:
                # Stance phase for first tripod
                self.control_leg('right_I', step_input, -speed)
                self.control_leg('left_II', step_input, -speed)
                self.control_leg('right_III', step_input, -speed)
                # Lift phase for second tripod if it's their turn
                if step_II == 1:
                    self.control_leg('left_I', step_II, speed, lift_height)
                    self.control_leg('right_II', step_II, speed, lift_height)
                    self.control_leg('left_III', step_II, speed, lift_height)
                else:
                    self.control_leg('left_I', step_II, -speed)
                    self.control_leg('right_II', step_II, -speed)
                    self.control_leg('left_III', step_II, -speed)
                    
        elif command == 'left':
            # Similar pattern but with opposite directions for left/right legs
            if step_input == 1:
                self.control_leg('right_I', step_input, speed, lift_height)
                self.control_leg('left_II', step_input, -speed, lift_height)
                self.control_leg('right_III', step_input, speed, lift_height)
                self.control_leg('left_I', step_II, -speed)
                self.control_leg('right_II', step_II, speed)
                self.control_leg('left_III', step_II, -speed)
            else:
                self.control_leg('right_I', step_input, -speed)
                self.control_leg('left_II', step_input, speed)
                self.control_leg('right_III', step_input, -speed)
                if step_II == 1:
                    self.control_leg('left_I', step_II, -speed, lift_height)
                    self.control_leg('right_II', step_II, speed, lift_height)
                    self.control_leg('left_III', step_II, -speed, lift_height)
                else:
                    self.control_leg('left_I', step_II, speed)
                    self.control_leg('right_II', step_II, -speed)
                    self.control_leg('left_III', step_II, speed)
                    
        elif command == 'right':
            # Mirror of left turn
            if step_input == 1:
                self.control_leg('right_I', step_input, -speed, lift_height)
                self.control_leg('left_II', step_input, speed, lift_height)
                self.control_leg('right_III', step_input, -speed, lift_height)
                self.control_leg('left_I', step_II, speed)
                self.control_leg('right_II', step_II, -speed)
                self.control_leg('left_III', step_II, speed)
            else:
                self.control_leg('right_I', step_input, speed)
                self.control_leg('left_II', step_input, -speed)
                self.control_leg('right_III', step_input, speed)
                if step_II == 1:
                    self.control_leg('left_I', step_II, speed, lift_height)
                    self.control_leg('right_II', step_II, -speed, lift_height)
                    self.control_leg('left_III', step_II, speed, lift_height)
                else:
                    self.control_leg('left_I', step_II, -speed)
                    self.control_leg('right_II', step_II, speed)
                    self.control_leg('left_III', step_II, -speed)

    def dove(self, step_input: int, speed: int, timeLast: float, dpi: int, command: str) -> None:
        """Execute a dove movement sequence.
        
        Args:
            step_input: Current step in sequence (1-4)
            speed: Movement speed (positive for forward, negative for backward)
            timeLast: Time to sleep between movements
            dpi: Steps per movement
            command: Movement command (no, left, right)
        """
        logger.debug(f"Dove movement - step: {step_input}, speed: {speed}, command: {command}")
        
        def dove_control_leg(leg_id: str, horizontal: int, vertical: int) -> None:
            """Control a single leg's dove movement."""
            leg = self.LEG_MAP[leg_id]
            
            # For left legs
            if leg_id.startswith('left'):
                # For backward movement (speed < 0), invert the horizontal direction
                direction = leg['direction'] if speed > 0 else not leg['direction']
                if direction:  # leftSide_direction == 1
                    self.sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] + horizontal)
                else:  # leftSide_direction == 0
                    self.sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] - horizontal)
                
                if leg['height']:  # leftSide_height == 1
                    self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + vertical)
                else:  # leftSide_height == 0
                    self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - vertical)
            
            # For right legs
            else:
                # For backward movement (speed < 0), invert the horizontal direction
                direction = leg['direction'] if speed > 0 else not leg['direction']
                if direction:  # rightSide_direction == 1
                    self.sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] + horizontal)
                else:  # rightSide_direction == 0
                    self.sc.set_servo_pwm(leg['horiz'], leg['pwm_h'] - horizontal)
                
                if leg['height']:  # rightSide_height == 1
                    self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] + vertical)
                else:  # rightSide_height == 0
                    self.sc.set_servo_pwm(leg['vert'], leg['pwm_v'] - vertical)
        
        step_II = step_input + 2
        if step_II > 4:
            step_II = step_II - 4

        # Use absolute speed value for calculations, direction is handled in dove_control_leg
        abs_speed = abs(speed)
        
        if step_input == 1:
            for speed_I in range(0, (abs_speed + int(abs_speed / dpi)), int(abs_speed / dpi)):
                if self.move_stu and command == 'no':
                    speed_II = speed_I
                    speed_I = abs_speed - speed_I
                    # First tripod moves up and forward
                    dove_control_leg('left_I', -speed_I, 3 * speed_II)
                    dove_control_leg('right_II', -speed_I, 3 * speed_II)
                    dove_control_leg('left_III', -speed_I, 3 * speed_II)
                    # Second tripod stays down and moves backward
                    dove_control_leg('right_I', speed_I, -10)
                    dove_control_leg('left_II', speed_I, -10)
                    dove_control_leg('right_III', speed_I, -10)
                    time.sleep(timeLast / dpi)
                elif command == 'left':
                    speed_II = speed_I
                    speed_I = abs_speed - speed_I
                    dove_control_leg('left_I', speed_I, 3 * speed_II)
                    dove_control_leg('right_II', -speed_I, 3 * speed_II)
                    dove_control_leg('left_III', speed_I, 3 * speed_II)

                    dove_control_leg('right_I', speed_I, -10)
                    dove_control_leg('left_II', -speed_I, -10)
                    dove_control_leg('right_III', speed_I, -10)
                    time.sleep(timeLast / dpi)
                elif command == 'right':
                    speed_II = speed_I
                    speed_I = abs_speed - speed_I
                    dove_control_leg('left_I', -speed_I, 3 * speed_II)
                    dove_control_leg('right_II', speed_I, 3 * speed_II)
                    dove_control_leg('left_III', -speed_I, 3 * speed_II)

                    dove_control_leg('right_I', -speed_I, -10)
                    dove_control_leg('left_II', speed_I, -10)
                    dove_control_leg('right_III', -speed_I, -10)
                    time.sleep(timeLast / dpi)

                if self.move_stu == 0 and command == 'no':
                    break

        elif step_input == 2:
            for speed_I in range(0, (abs_speed + int(abs_speed / dpi)), int(abs_speed / dpi)):
                if self.move_stu and command == 'no':
                    speed_II = speed_I
                    speed_I = abs_speed - speed_I
                    # First tripod moves down and back
                    dove_control_leg('left_I', speed_II, 3 * (abs_speed - speed_II))
                    dove_control_leg('right_II', speed_II, 3 * (abs_speed - speed_II))
                    dove_control_leg('left_III', speed_II, 3 * (abs_speed - speed_II))
                    # Second tripod moves forward
                    dove_control_leg('right_I', -speed_II, -10)
                    dove_control_leg('left_II', -speed_II, -10)
                    dove_control_leg('right_III', -speed_II, -10)
                    time.sleep(timeLast / dpi)

                if self.move_stu == 0 and command == 'no':
                    break

        elif step_input == 3:
            for speed_I in range(0, (abs_speed + int(abs_speed / dpi)), int(abs_speed / dpi)):
                if self.move_stu and command == 'no':
                    speed_II = speed_I
                    speed_I = abs_speed - speed_I
                    # First tripod stays down
                    dove_control_leg('left_I', speed_I, -10)
                    dove_control_leg('right_II', speed_I, -10)
                    dove_control_leg('left_III', speed_I, -10)
                    # Second tripod lifts and moves forward
                    dove_control_leg('right_I', -speed_I, 3 * speed_II)
                    dove_control_leg('left_II', -speed_I, 3 * speed_II)
                    dove_control_leg('right_III', -speed_I, 3 * speed_II)
                    time.sleep(timeLast / dpi)

                if self.move_stu == 0 and command == 'no':
                    break

        elif step_input == 4:
            for speed_I in range(0, (abs_speed + int(abs_speed / dpi)), int(abs_speed / dpi)):
                if self.move_stu and command == 'no':
                    speed_II = speed_I
                    speed_I = abs_speed - speed_I
                    # First tripod moves forward
                    dove_control_leg('left_I', -speed_II, -10)
                    dove_control_leg('right_II', -speed_II, -10)
                    dove_control_leg('left_III', -speed_II, -10)
                    # Second tripod moves down and back
                    dove_control_leg('right_I', speed_II, 3 * (abs_speed - speed_II))
                    dove_control_leg('left_II', speed_II, 3 * (abs_speed - speed_II))
                    dove_control_leg('right_III', speed_II, 3 * (abs_speed - speed_II))
                    time.sleep(timeLast / dpi)

                if self.move_stu == 0 and command == 'no':
                    break

    def _move_thread(self) -> None:
        """Internal movement thread function."""
        if not self.steadyMode:
            logger.debug(f"Movement state - Direction: {self.direction_command}, Turn: {self.turn_command}, Step: {self.step_set}")
            
            # Check if we're completing a movement cycle
            if self.completing_movement:
                # Continue the current movement until we reach step 1 (initial position)
                if self.direction_command in ['forward', 'backward']:
                    speed = self.DOVE_SPEED if self.direction_command == 'forward' else -self.DOVE_SPEED
                    if self.SmoothMode:
                        logger.debug(f"Completing {self.direction_command} movement - Speed: {speed}, Step: {self.step_set}")
                        self.dove(self.step_set, speed, 0.001, self.DPI, 'no')
                    else:
                        speed = 35 if self.direction_command == 'forward' else -35
                        logger.debug(f"Completing {self.direction_command} movement - Speed: {speed}, Step: {self.step_set}")
                        self.move(self.step_set, speed, 'no')
                        time.sleep(0.1)
                    
                    # Update step and check if we've completed the cycle
                    self.step_set = (self.step_set % 4) + 1
                    if self.step_set == 1:
                        self.completing_movement = False
                        self.direction_command = 'no'
                        self.turn_command = 'no'
                        self.movement_thread.pause()
                        logger.info("Movement cycle completed, stopping at initial position")
                        return
                
                elif self.turn_command in ['left', 'right']:
                    if self.SmoothMode:
                        logger.debug(f"Completing {self.turn_command} turn - Step: {self.step_set}")
                        self.dove(self.step_set, 35, 0.001, self.DPI, self.turn_command)
                    else:
                        logger.debug(f"Completing {self.turn_command} turn - Step: {self.step_set}")
                        self.move(self.step_set, 35, self.turn_command)
                        time.sleep(0.1)
                    
                    # Update step and check if we've completed the cycle
                    self.step_set = (self.step_set % 4) + 1
                    if self.step_set == 1:
                        self.completing_movement = False
                        self.direction_command = 'no'
                        self.turn_command = 'no'
                        self.movement_thread.pause()
                        logger.info("Turn cycle completed, stopping at initial position")
                        return
            
            # Normal movement processing
            elif self.direction_command in ['forward', 'backward']:
                speed = self.DOVE_SPEED if self.direction_command == 'forward' else -self.DOVE_SPEED
                if self.SmoothMode:
                    logger.debug(f"Smooth {self.direction_command} movement - Speed: {speed}, Step: {self.step_set}")
                    self.dove(self.step_set, speed, 0.001, self.DPI, 'no')
                else:
                    speed = 35 if self.direction_command == 'forward' else -35
                    logger.debug(f"Normal {self.direction_command} movement - Speed: {speed}, Step: {self.step_set}")
                    self.move(self.step_set, speed, 'no')
                    time.sleep(0.1)
                
                self.step_set = (self.step_set % 4) + 1
                
            elif self.turn_command in ['left', 'right']:
                if self.SmoothMode:
                    logger.debug(f"Smooth {self.turn_command} turn - Step: {self.step_set}")
                    self.dove(self.step_set, 35, 0.001, self.DPI, self.turn_command)
                else:
                    logger.debug(f"Normal {self.turn_command} turn - Step: {self.step_set}")
                    self.move(self.step_set, 35, self.turn_command)
                    time.sleep(0.1)
                
                self.step_set = (self.step_set % 4) + 1

            elif self.direction_command == 'stand':
                logger.debug("Moving to stand position")
                self.stand()
                self.step_set = 1
        else:
            logger.debug("Steady mode active")
            self.steady_X()
            if hasattr(self, 'mpu_sensor') and self.mpu_sensor is not None:
                self.steady(self.mpu_sensor)
            else:
                logger.warning("No MPU sensor available for steady mode")

    def steady_X(self) -> None:
        """Adjust X-axis balance for steady mode."""
        # Front legs
        front_left = self.LEG_MAP['left_I']
        front_right = self.LEG_MAP['right_III']
        
        # Middle legs
        middle_left = self.LEG_MAP['left_II']
        middle_right = self.LEG_MAP['right_II']
        
        # Back legs
        back_left = self.LEG_MAP['left_III']
        back_right = self.LEG_MAP['right_I']
        
        # Left side
        if front_left['direction']:
            self.sc.set_servo_pwm(front_left['horiz'], front_left['pwm_h'] + self.steady_X_set)
            self.sc.set_servo_pwm(middle_left['horiz'], middle_left['pwm_h'])
            self.sc.set_servo_pwm(back_left['horiz'], back_left['pwm_h'] - self.steady_X_set)
        else:
            self.sc.set_servo_pwm(front_left['horiz'], front_left['pwm_h'] - self.steady_X_set)
            self.sc.set_servo_pwm(middle_left['horiz'], middle_left['pwm_h'])
            self.sc.set_servo_pwm(back_left['horiz'], back_left['pwm_h'] + self.steady_X_set)

        # Right side
        if front_right['direction']:
            self.sc.set_servo_pwm(front_right['horiz'], front_right['pwm_h'] + self.steady_X_set)
            self.sc.set_servo_pwm(middle_right['horiz'], middle_right['pwm_h'])
            self.sc.set_servo_pwm(back_right['horiz'], back_right['pwm_h'] - self.steady_X_set)
        else:
            self.sc.set_servo_pwm(front_right['horiz'], front_right['pwm_h'] - self.steady_X_set)
            self.sc.set_servo_pwm(middle_right['horiz'], middle_right['pwm_h'])
            self.sc.set_servo_pwm(back_right['horiz'], back_right['pwm_h'] + self.steady_X_set)

    def steady(self, mpu_sensor) -> None:
        """Maintain steady position using MPU sensor.
        
        Args:
            mpu_sensor: MPU6050 sensor instance for reading accelerometer data
        """
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
                if leg_id == 'left_I':
                    input_val = self.ctrl_range((self.X_fix_output + self.Y_fix_output), 
                                             self.steady_range_Max, self.steady_range_Min)
                elif leg_id == 'left_II':
                    input_val = self.ctrl_range((abs(self.X_fix_output * 0.5) + self.Y_fix_output), 
                                             self.steady_range_Max, self.steady_range_Min)
                else:  # left_III
                    input_val = self.ctrl_range((-self.X_fix_output + self.Y_fix_output), 
                                             self.steady_range_Max, self.steady_range_Min)
            else:  # right legs
                if leg_id == 'right_III':
                    input_val = self.ctrl_range((self.X_fix_output - self.Y_fix_output), 
                                             self.steady_range_Max, self.steady_range_Min)
                elif leg_id == 'right_II':
                    input_val = self.ctrl_range((abs(-self.X_fix_output * 0.5) - self.Y_fix_output), 
                                             self.steady_range_Max, self.steady_range_Min)
                else:  # right_I
                    input_val = self.ctrl_range((-self.X_fix_output - self.Y_fix_output), 
                                             self.steady_range_Max, self.steady_range_Min)
            
            self.control_leg(leg_id, 0, 35, input_val)

    def ctrl_range(self, raw: float, max_genout: float, min_genout: float) -> int:
        """Control the range of a value.
        
        Args:
            raw: Raw input value
            max_genout: Maximum allowed value
            min_genout: Minimum allowed value
            
        Returns:
            Value clamped to the specified range
        """
        if raw > max_genout:
            raw_output = max_genout
        elif raw < min_genout:
            raw_output = min_genout
        else:
            raw_output = raw
        return int(raw_output)

    def stand(self) -> None:
        """Put robot in standing position."""
        logger.info("Moving to standing position")
        for leg_name, leg_config in self.config.items():
            if leg_name != 'limits' and isinstance(leg_config, dict) and 'channels' in leg_config:
                channels = leg_config['channels']
                center = leg_config['center_position']
                self.sc.set_servo_pwm(channels['horizontal'], center['horizontal'])
                self.sc.set_servo_pwm(channels['vertical'], center['vertical'])

    def release(self) -> None:
        """Release all servos to neutral position."""
        logger.info("Releasing servos to neutral position")
        for leg_name, leg_config in self.config.items():
            if leg_name != 'limits' and isinstance(leg_config, dict) and 'channels' in leg_config:
                channels = leg_config['channels']
                center = leg_config['center_position']
                self.sc.set_servo_pwm(channels['horizontal'], center['horizontal'])
                self.sc.set_servo_pwm(channels['vertical'], center['vertical'])

    def clean_all(self) -> None:
        """Clean up all servo positions."""
        logger.info("Cleaning up servo positions")
        self.release()

    def destroy(self) -> None:
        """Cleanup before destroying the instance."""
        logger.info("Destroying LegsMovement instance")
        self.clean_all()

    def set_mpu_sensor(self, sensor) -> None:
        """Set the MPU sensor for steady mode.
        
        Args:
            sensor: MPU6050 sensor instance
        """
        self.mpu_sensor = sensor
        logger.info("MPU sensor set for steady mode")