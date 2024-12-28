import time
import threading
import logging
from typing import Dict, Optional

from servo import base
from system.kalman_filter import KalmanFilter
import PID

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RobotMovement:
    """Main class for controlling robot movement.
    This class handles all servo movements including walking, turning, camera control,
    and stabilization using PID controllers.
    """

    def __init__(self, servo_controller: base.ServoCtrl):
        """Initialize robot movement controller.
        
        Args:
            servo_controller: ServoCtrl instance for controlling servos
        """
        logger.info("RobotMovement: initializing")
        
        # Store servo controller
        self.sc = servo_controller
        
        # Movement state
        self.direction_command = 'no'
        self.turn_command = 'no'
        self.step_set = 1
        self.move_stu = 1
        
        # Movement modes
        self.smooth_mode = True
        self.steady_mode = False
        
        # Configuration
        self.set_direction = True  # Change to False to reverse all servos
        self._init_direction_config()
        self._init_movement_params()
        self._init_pid_controllers()
        
        # Initialize servo positions
        self.init_pwms = None
        self.pwm_values = {}  # Will store PWM values for each servo
        
        # Initialize movement thread
        self._thread_event = threading.Event()
        self._movement_thread = None
        self._start_movement_thread()

    def _init_direction_config(self):
        """Initialize direction configuration for legs and camera."""
        # Leg direction configuration
        self.left_side_direction = True if self.set_direction else False
        self.right_side_direction = False if self.set_direction else True
        
        # Leg height configuration
        self.left_side_height = False if self.set_direction else True
        self.right_side_height = True if self.set_direction else False
        
        # Camera direction configuration
        self.up_down_direction = True if self.set_direction else False
        self.left_right_direction = True if self.set_direction else False

    def _init_movement_params(self):
        """Initialize movement parameters."""
        # Height adjustment
        self.height_change = 30
        
        # Camera movement parameters
        self.left_right_input = 300
        self.up_down_input = 300
        self.left_right_max = 500
        self.left_right_min = 100
        self.up_down_max = 500
        self.up_down_min = 270
        self.look_wiggle = 30
        
        # Movement speeds
        self.dove_speed = 20
        self.dpi = 17  # Smoothing factor for dove movement
        
        # Steady mode parameters
        self.steady_range_min = -40
        self.steady_range_max = 130
        self.range_mid = (self.steady_range_min + self.steady_range_max) / 2
        self.x_fix_output = self.range_mid
        self.y_fix_output = self.range_mid
        self.steady_x_set = 73

    def _init_pid_controllers(self):
        """Initialize PID controllers and Kalman filters for stabilization."""
        # PID parameters
        self.p = 5
        self.i = 0.01
        self.d = 0
        
        # PID controllers
        self.x_pid = PID.PID()
        self.x_pid.SetKp(self.p)
        self.x_pid.SetKd(self.i)
        self.x_pid.SetKi(self.d)
        
        self.y_pid = PID.PID()
        self.y_pid.SetKp(self.p)
        self.y_pid.SetKd(self.i)
        self.y_pid.SetKi(self.d)
        
        # Kalman filters
        self.kalman_filter_x = KalmanFilter(0.001, 0.1)
        self.kalman_filter_y = KalmanFilter(0.001, 0.1)
        
        # Target values
        self.target_x = 0
        self.target_y = 0

    def set_init_positions(self, positions: Dict[int, int]):
        """Set initial positions for all servos.
        
        Args:
            positions: Dictionary mapping servo numbers to their initial positions
        """
        logger.info("Setting initial servo positions")
        self.init_pwms = positions.copy()
        
        # Store individual PWM values
        for i in range(16):
            self.pwm_values[i] = self.init_pwms[i]
        
        # Initialize all servos to their positions
        self.init_all()

    def init_all(self):
        """Initialize all servos to their starting positions."""
        logger.info("Initializing all servos to starting positions")
        for servo_num, position in self.pwm_values.items():
            self.sc.set_servo_pwm(servo_num, position)

    def _ctrl_range(self, raw: float, max_val: float, min_val: float) -> int:
        """Control the range of raw values.
        
        Args:
            raw: Input value
            max_val: Maximum allowed value
            min_val: Minimum allowed value
            
        Returns:
            Value clamped to the allowed range
        """
        return int(max(min(raw, max_val), min_val))

    def _move_leg(self, leg_func, pos: int, wiggle: int, height_adjust: int = 0):
        """Generic function for moving a leg.
        
        Args:
            leg_func: Function that implements specific leg movement
            pos: Position step (0-4)
            wiggle: Amount of wiggle
            height_adjust: Height adjustment
        """
        leg_func(pos, wiggle, height_adjust)

    def move_left_leg(self, leg_num: int, pos: int, wiggle: int, height_adjust: int = 0):
        """Move a left leg.
        
        Args:
            leg_num: Leg number (1-3)
            pos: Position step (0-4)
            wiggle: Amount of wiggle
            height_adjust: Height adjustment
        """
        servo_base = (leg_num - 1) * 2  # 0, 2, or 4 for legs 1, 2, 3
        
        if pos == 0:
            if self.left_side_height:
                self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height_adjust)
            else:
                self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] - height_adjust)
            return

        if self.left_side_direction:
            self._move_left_leg_forward(servo_base, pos, wiggle)
        else:
            self._move_left_leg_backward(servo_base, pos, wiggle)

    def _move_left_leg_forward(self, servo_base: int, pos: int, wiggle: int):
        """Implementation of forward movement for left leg."""
        if pos == 1:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base])
            height = 3 * self.height_change if self.left_side_height else -3 * self.height_change
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height)
        elif pos == 2:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] + wiggle)
            height = -self.height_change if self.left_side_height else self.height_change
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height)
        # ... similar for pos 3 and 4

    def _move_left_leg_backward(self, servo_base: int, pos: int, wiggle: int):
        """Implementation of backward movement for left leg."""
        if pos == 1:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base])
            height = 3 * self.height_change if self.left_side_height else -3 * self.height_change
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height)
        elif pos == 2:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] - wiggle)
            height = -self.height_change if self.left_side_height else self.height_change
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height)
        elif pos == 3:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base])
            height = -self.height_change if self.left_side_height else self.height_change
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height)
        elif pos == 4:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] + wiggle)
            height = -self.height_change if self.left_side_height else self.height_change
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height)

    def move_right_leg(self, leg_num: int, pos: int, wiggle: int, height_adjust: int = 0):
        """Move a right leg.
        
        Args:
            leg_num: Leg number (1-3)
            pos: Position step (0-4)
            wiggle: Amount of wiggle
            height_adjust: Height adjustment
        """
        servo_base = 6 + (leg_num - 1) * 2  # 6, 8, or 10 for legs 1, 2, 3
        
        if pos == 0:
            if self.right_side_height:
                self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height_adjust)
            else:
                self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] - height_adjust)
            return

        if self.right_side_direction:
            self._move_right_leg_forward(servo_base, pos, wiggle)
        else:
            self._move_right_leg_backward(servo_base, pos, wiggle)

    def _move_right_leg_forward(self, servo_base: int, pos: int, wiggle: int):
        """Implementation of forward movement for right leg."""
        if pos == 1:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base])
            height = 3 * self.height_change if self.right_side_height else -3 * self.height_change
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height)
        elif pos == 2:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] + wiggle)
            height = -self.height_change if self.right_side_height else self.height_change
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height)
        elif pos == 3:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base])
            height = -self.height_change if self.right_side_height else self.height_change
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height)
        elif pos == 4:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] - wiggle)
            height = -self.height_change if self.right_side_height else self.height_change
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height)

    def _move_right_leg_backward(self, servo_base: int, pos: int, wiggle: int):
        """Implementation of backward movement for right leg."""
        if pos == 1:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base])
            height = 3 * self.height_change if self.right_side_height else -3 * self.height_change
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height)
        elif pos == 2:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] - wiggle)
            height = -self.height_change if self.right_side_height else self.height_change
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height)
        elif pos == 3:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base])
            height = -self.height_change if self.right_side_height else self.height_change
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height)
        elif pos == 4:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] + wiggle)
            height = -self.height_change if self.right_side_height else self.height_change
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + height)

    def move(self, step: int, speed: int, command: str = 'no'):
        """Execute a movement step.
        
        Args:
            step: Current step number (1-4)
            speed: Movement speed
            command: Movement command ('no', 'left', 'right')
        """
        logger.info(f"Moving: step={step}, speed={speed}, command={command}")
        
        step_1 = step
        step_2 = (step + 2) % 4 or 4  # Keep in range 1-4
        
        if speed == 0:
            return

        if command == 'no':
            self._move_straight(step_1, step_2, speed)
        elif command == 'left':
            self._move_left(step_1, step_2, speed)
        elif command == 'right':
            self._move_right(step_1, step_2, speed)

    def _move_straight(self, step_1: int, step_2: int, speed: int):
        """Execute straight movement."""
        # First tripod
        self.move_right_leg(1, step_1, speed)
        self.move_left_leg(2, step_1, speed)
        self.move_right_leg(3, step_1, speed)
        
        # Second tripod
        self.move_left_leg(1, step_2, speed)
        self.move_right_leg(2, step_2, speed)
        self.move_left_leg(3, step_2, speed)

    def stand(self):
        """Make the robot stand (neutral position)."""
        logger.info("Standing")
        for i in range(12):  # Only the leg servos
            self.sc.set_servo_pwm(i, 300)

    def cleanup(self):
        """Clean up and set all servos to neutral position."""
        logger.info("Cleaning up")
        self._thread_event.clear()
        if self._movement_thread:
            self._movement_thread.join()
        
        for i in range(16):
            self.sc.set_servo_pwm(i, 300)

    def _start_movement_thread(self):
        """Start the movement control thread."""
        self._movement_thread = threading.Thread(target=self._movement_loop, daemon=True)
        self._movement_thread.start()

    def _movement_loop(self):
        """Main movement control loop."""
        while True:
            self._thread_event.wait()
            
            if not self.steady_mode:
                self._handle_movement()
            else:
                self._handle_steady()
            
            time.sleep(0.02)  # Prevent CPU overuse

    def _handle_movement(self):
        """Handle movement based on current commands."""
        if self.direction_command == 'forward' and self.turn_command == 'no':
            if self.smooth_mode:
                self.dove(self.step_set, self.dove_speed, 0.001, self.dpi, 'no')
            else:
                self.move(self.step_set, 35, 'no')
                time.sleep(0.1)
            
            self.step_set = (self.step_set % 4) + 1

        elif self.direction_command == 'backward' and self.turn_command == 'no':
            if self.smooth_mode:
                self.dove(self.step_set, -self.dove_speed, 0.001, self.dpi, 'no')
            else:
                self.move(self.step_set, -35, 'no')
                time.sleep(0.1)
            
            self.step_set = (self.step_set % 4) + 1

        elif self.turn_command != 'no':
            if self.smooth_mode:
                self.dove(self.step_set, 35, 0.001, self.dpi, self.turn_command)
            else:
                self.move(self.step_set, 35, self.turn_command)
                time.sleep(0.1)
            
            self.step_set = (self.step_set % 4) + 1

        elif self.turn_command == 'no' and self.direction_command == 'stand':
            self.stand()
            self.step_set = 1

    def command(self, cmd: str):
        """Process movement commands.
        
        Args:
            cmd: Command string ('forward', 'backward', 'left', 'right', 'stand', etc.)
        """
        logger.info(f"Processing command: {cmd}")
        
        if cmd == 'forward':
            self.direction_command = 'forward'
            self._thread_event.set()
        
        elif cmd == 'backward':
            self.direction_command = 'backward'
            self._thread_event.set()
        
        elif cmd == 'stand':
            self.direction_command = 'stand'
            self._thread_event.clear()
        
        elif cmd == 'left':
            self.turn_command = 'left'
            self._thread_event.set()
        
        elif cmd == 'right':
            self.turn_command = 'right'
            self._thread_event.set()
        
        elif cmd == 'no':
            self.turn_command = 'no'
            self._thread_event.clear()
        
        elif cmd == 'automaticOff':
            self.smooth_mode = False
            self.steady_mode = False
            self._thread_event.clear()
        
        elif cmd == 'automatic':
            self.smooth_mode = True
            self._thread_event.set()
        
        elif cmd in ['KD', 'speech']:
            self.steady_mode = True
            self._thread_event.set()
        
        elif cmd == 'speechOff':
            self.smooth_mode = False
            self.steady_mode = False
            self._thread_event.clear()

    # Camera control methods
    def look_up(self, wiggle: Optional[int] = None):
        """Look up by adjusting camera servo."""
        wiggle = wiggle or self.look_wiggle
        if self.up_down_direction:
            self.up_down_input += wiggle
        else:
            self.up_down_input -= wiggle
        
        self.up_down_input = self._ctrl_range(
            self.up_down_input, 
            self.up_down_max, 
            self.up_down_min
        )
        self.sc.set_servo_pwm(13, self.up_down_input)

    def look_down(self, wiggle: Optional[int] = None):
        """Look down by adjusting camera servo."""
        wiggle = wiggle or self.look_wiggle
        if self.up_down_direction:
            self.up_down_input -= wiggle
        else:
            self.up_down_input += wiggle
        
        self.up_down_input = self._ctrl_range(
            self.up_down_input, 
            self.up_down_max, 
            self.up_down_min
        )
        self.sc.set_servo_pwm(13, self.up_down_input)

    def look_left(self, wiggle: Optional[int] = None):
        """Look left by adjusting camera servo."""
        wiggle = wiggle or self.look_wiggle
        if self.left_right_direction:
            self.left_right_input += wiggle
        else:
            self.left_right_input -= wiggle
        
        self.left_right_input = self._ctrl_range(
            self.left_right_input, 
            self.left_right_max, 
            self.left_right_min
        )
        self.sc.set_servo_pwm(12, self.left_right_input)

    def look_right(self, wiggle: Optional[int] = None):
        """Look right by adjusting camera servo."""
        wiggle = wiggle or self.look_wiggle
        if self.left_right_direction:
            self.left_right_input -= wiggle
        else:
            self.left_right_input += wiggle
        
        self.left_right_input = self._ctrl_range(
            self.left_right_input, 
            self.left_right_max, 
            self.left_right_min
        )
        self.sc.set_servo_pwm(12, self.left_right_input)

    def look_home(self):
        """Return camera to home position."""
        logger.info("Moving camera to home position")
        self.sc.set_servo_pwm(13, 300)
        self.sc.set_servo_pwm(12, 300)
        self.left_right_input = 300
        self.up_down_input = 300

    def _move_left(self, step_1: int, step_2: int, speed: int):
        """Execute left turn movement."""
        # First tripod with adjusted speeds
        self.move_right_leg(1, step_1, speed)
        self.move_left_leg(2, step_1, -speed)
        self.move_right_leg(3, step_1, speed)
        
        # Second tripod with adjusted speeds
        self.move_left_leg(1, step_2, -speed)
        self.move_right_leg(2, step_2, speed)
        self.move_left_leg(3, step_2, -speed)

    def _move_right(self, step_1: int, step_2: int, speed: int):
        """Execute right turn movement."""
        # First tripod with adjusted speeds
        self.move_right_leg(1, step_1, -speed)
        self.move_left_leg(2, step_1, speed)
        self.move_right_leg(3, step_1, -speed)
        
        # Second tripod with adjusted speeds
        self.move_left_leg(1, step_2, speed)
        self.move_right_leg(2, step_2, -speed)
        self.move_left_leg(3, step_2, speed)

    def dove(self, step_input: int, speed: int, time_last: float, dpi: int, command: str = 'no'):
        """Execute smooth movement using dove algorithm.
        
        Args:
            step_input: Current step (1-4)
            speed: Movement speed
            time_last: Time delay between movements
            dpi: Smoothing factor
            command: Movement command ('no', 'left', 'right')
        """
        logger.info(f"Dove movement: step={step_input}, speed={speed}, command={command}")
        
        if speed == 0:
            return

        step_increment = int(speed / dpi)
        
        for speed_i in range(0, speed + step_increment, step_increment):
            speed_ii = speed_i
            speed_i = speed - speed_i
            
            if step_input == 1:
                self._dove_step_1(speed_i, speed_ii, command, time_last, dpi)
            elif step_input == 2:
                self._dove_step_2(speed_i, speed_ii, command, time_last, dpi)
            elif step_input == 3:
                self._dove_step_3(speed_i, speed_ii, command, time_last, dpi)
            elif step_input == 4:
                self._dove_step_4(speed_i, speed_ii, command, time_last, dpi)

    def _dove_step_1(self, speed_i: int, speed_ii: int, command: str, time_last: float, dpi: int):
        """Execute first step of dove movement."""
        if command == 'no':
            self._dove_legs_group1(-speed_i, 3 * speed_ii)
            self._dove_legs_group2(speed_i, -10)
        elif command == 'left':
            self._dove_legs_group1_left(speed_i, 3 * speed_ii)
            self._dove_legs_group2_left(speed_i, -10)
        elif command == 'right':
            self._dove_legs_group1_right(-speed_i, 3 * speed_ii)
            self._dove_legs_group2_right(-speed_i, -10)
        time.sleep(time_last / dpi)

    def _dove_step_2(self, speed_i: int, speed_ii: int, command: str, time_last: float, dpi: int):
        """Execute second step of dove movement."""
        if command == 'no':
            self._dove_legs_group1(speed_ii, 3 * (speed_i - speed_ii))
            self._dove_legs_group2(-speed_ii, -10)
        elif command == 'left':
            self._dove_legs_group1_left(-speed_ii, 3 * (speed_i - speed_ii))
            self._dove_legs_group2_left(speed_ii, -10)
        elif command == 'right':
            self._dove_legs_group1_right(speed_ii, 3 * (speed_i - speed_ii))
            self._dove_legs_group2_right(-speed_ii, -10)
        time.sleep(time_last / dpi)

    def _dove_step_3(self, speed_i: int, speed_ii: int, command: str, time_last: float, dpi: int):
        """Execute third step of dove movement."""
        if command == 'no':
            self._dove_legs_group1(speed_i, -10)
            self._dove_legs_group2(-speed_i, 3 * speed_ii)
        elif command == 'left':
            self._dove_legs_group1_left(-speed_i, -10)
            self._dove_legs_group2_left(speed_i, 3 * speed_ii)
        elif command == 'right':
            self._dove_legs_group1_right(speed_i, -10)
            self._dove_legs_group2_right(speed_i, 3 * speed_ii)
        time.sleep(time_last / dpi)

    def _dove_step_4(self, speed_i: int, speed_ii: int, command: str, time_last: float, dpi: int):
        """Execute fourth step of dove movement."""
        if command == 'no':
            self._dove_legs_group1(-speed_ii, -10)
            self._dove_legs_group2(speed_ii, 3 * (speed_i - speed_ii))
        elif command == 'left':
            self._dove_legs_group1_left(speed_ii, -10)
            self._dove_legs_group2_left(-speed_ii, 3 * (speed_i - speed_ii))
        elif command == 'right':
            self._dove_legs_group1_right(-speed_ii, -10)
            self._dove_legs_group2_right(speed_ii, 3 * (speed_i - speed_ii))
        time.sleep(time_last / dpi)

    def _dove_legs_group1(self, horizontal: int, vertical: int):
        """Move first group of legs in dove movement."""
        self._dove_left_leg(0, horizontal, vertical)   # Left I
        self._dove_right_leg(1, horizontal, vertical)  # Right II
        self._dove_left_leg(2, horizontal, vertical)   # Left III

    def _dove_legs_group2(self, horizontal: int, vertical: int):
        """Move second group of legs in dove movement."""
        self._dove_right_leg(0, horizontal, vertical)  # Right I
        self._dove_left_leg(1, horizontal, vertical)   # Left II
        self._dove_right_leg(2, horizontal, vertical)  # Right III

    def _dove_left_leg(self, leg_num: int, horizontal: int, vertical: int):
        """Move a left leg in dove movement."""
        servo_base = leg_num * 2
        if self.left_side_direction:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] + horizontal)
        else:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] - horizontal)

        if self.left_side_height:
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + vertical)
        else:
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] - vertical)

    def _dove_right_leg(self, leg_num: int, horizontal: int, vertical: int):
        """Move a right leg in dove movement."""
        servo_base = 6 + leg_num * 2
        if self.right_side_direction:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] + horizontal)
        else:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] - horizontal)

        if self.right_side_height:
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + vertical)
        else:
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] - vertical)

    def _handle_steady(self):
        """Handle steady mode movement."""
        self._steady_x()
        self.steady()

    def _steady_x(self):
        """Adjust X-axis for steady movement."""
        if self.left_side_direction:
            self.sc.set_servo_pwm(0, self.pwm_values[0] + self.steady_x_set)
            self.sc.set_servo_pwm(2, self.pwm_values[2])
            self.sc.set_servo_pwm(4, self.pwm_values[4] - self.steady_x_set)
        else:
            self.sc.set_servo_pwm(0, self.pwm_values[0] + self.steady_x_set)
            self.sc.set_servo_pwm(2, self.pwm_values[2])
            self.sc.set_servo_pwm(4, self.pwm_values[4] - self.steady_x_set)

        if self.right_side_direction:
            self.sc.set_servo_pwm(10, self.pwm_values[10] + self.steady_x_set)
            self.sc.set_servo_pwm(8, self.pwm_values[8])
            self.sc.set_servo_pwm(6, self.pwm_values[6] - self.steady_x_set)
        else:
            self.sc.set_servo_pwm(10, self.pwm_values[10] - self.steady_x_set)
            self.sc.set_servo_pwm(8, self.pwm_values[8])
            self.sc.set_servo_pwm(6, self.pwm_values[6] + self.steady_x_set)

    def steady(self, mpu_sensor=None):
        """Execute steady movement using MPU sensor data."""
        if not mpu_sensor:
            return

        # Get and filter accelerometer data
        accel_data = mpu_sensor.get_accel_data()
        x = self.kalman_filter_x.kalman(accel_data['x'])
        y = self.kalman_filter_y.kalman(accel_data['y'])

        # Update PID outputs
        self.x_fix_output += -self.x_pid.GenOut(x - self.target_x)
        self.x_fix_output = self._ctrl_range(
            self.x_fix_output, 
            self.steady_range_max, 
            -self.steady_range_max
        )

        self.y_fix_output += -self.y_pid.GenOut(y - self.target_y)
        self.y_fix_output = self._ctrl_range(
            self.y_fix_output, 
            self.steady_range_max, 
            -self.steady_range_max
        )

        # Apply corrections to each leg
        self._steady_leg_adjustments()

    def _steady_leg_adjustments(self):
        """Apply steady mode adjustments to all legs."""
        # Left legs
        left_i_input = self._ctrl_range(
            (self.x_fix_output + self.y_fix_output),
            self.steady_range_max,
            self.steady_range_min
        )
        self.move_left_leg(1, 0, 35, left_i_input)

        left_ii_input = self._ctrl_range(
            (abs(self.x_fix_output * 0.5) + self.y_fix_output),
            self.steady_range_max,
            self.steady_range_min
        )
        self.move_left_leg(2, 0, 35, left_ii_input)

        left_iii_input = self._ctrl_range(
            (-self.x_fix_output + self.y_fix_output),
            self.steady_range_max,
            self.steady_range_min
        )
        self.move_left_leg(3, 0, 35, left_iii_input)

        # Right legs
        right_i_input = self._ctrl_range(
            (-self.x_fix_output - self.y_fix_output),
            self.steady_range_max,
            self.steady_range_min
        )
        self.move_right_leg(1, 0, 35, right_i_input)

        right_ii_input = self._ctrl_range(
            (abs(-self.x_fix_output * 0.5) - self.y_fix_output),
            self.steady_range_max,
            self.steady_range_min
        )
        self.move_right_leg(2, 0, 35, right_ii_input)

        right_iii_input = self._ctrl_range(
            (self.x_fix_output - self.y_fix_output),
            self.steady_range_max,
            self.steady_range_min
        )
        self.move_right_leg(3, 0, 35, right_iii_input)

    def _dove_legs_group1_left(self, horizontal: int, vertical: int):
        """Move first group of legs in left turn dove movement."""
        self._dove_left_leg(0, horizontal, vertical)   # Left I
        self._dove_right_leg(1, -horizontal, vertical)  # Right II
        self._dove_left_leg(2, horizontal, vertical)   # Left III

    def _dove_legs_group2_left(self, horizontal: int, vertical: int):
        """Move second group of legs in left turn dove movement."""
        self._dove_right_leg(0, horizontal, vertical)  # Right I
        self._dove_left_leg(1, -horizontal, vertical)  # Left II
        self._dove_right_leg(2, horizontal, vertical)  # Right III

    def _dove_legs_group1_right(self, horizontal: int, vertical: int):
        """Move first group of legs in right turn dove movement."""
        self._dove_left_leg(0, -horizontal, vertical)   # Left I
        self._dove_right_leg(1, horizontal, vertical)  # Right II
        self._dove_left_leg(2, -horizontal, vertical)   # Left III

    def _dove_legs_group2_right(self, horizontal: int, vertical: int):
        """Move second group of legs in right turn dove movement."""
        self._dove_right_leg(0, -horizontal, vertical)  # Right I
        self._dove_left_leg(1, horizontal, vertical)  # Left II
        self._dove_right_leg(2, -horizontal, vertical)  # Right III


