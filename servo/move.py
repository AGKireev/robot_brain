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
        """Initialize the robot movement controller.
        
        Args:
            servo_controller: ServoCtrl instance for controlling servos
        """
        self.sc = servo_controller
        self.pwm_values = {}  # Store PWM values for each servo
        
        # Direction controls - CRITICAL for correct movement
        self.set_direction = 1
        self.left_side_direction = 1 if self.set_direction else 0
        self.right_side_direction = 0 if self.set_direction else 1
        self.left_side_height = 0 if self.set_direction else 1
        self.right_side_height = 1 if self.set_direction else 0
        
        # Camera direction configuration
        self.up_down_direction = True if self.set_direction else False
        self.left_right_direction = True if self.set_direction else False
        
        # Camera movement parameters
        self.left_right_input = 300
        self.up_down_input = 300
        self.left_right_max = 500
        self.left_right_min = 100
        self.up_down_max = 500
        self.up_down_min = 270
        self.look_wiggle = 30
        
        # Movement parameters
        self.height_change = 30
        self.step_set = 1
        self.speed_set = 100
        self.dpi = 17
        self.dove_speed = 20
        self.smooth_mode = 1
        self.steady_mode = 0
        
        # Movement state
        self.direction_command = 'no'
        self.turn_command = 'no'
        self.move_stu = 1
        
        # Initialize PID controllers
        self._init_pid_controllers()
        
        # Initialize threading
        self._thread_event = threading.Event()
        self._movement_thread = None
        
        # Start thread only after everything is initialized
        self._start_movement_thread()

        # Add missing variables so that steady mode & turning logic matches move_old.py
        self.steady_range_min = -40
        self.steady_range_max = 130
        self.range_mid = (self.steady_range_min + self.steady_range_max) / 2
        self.x_fix_output = self.range_mid
        self.y_fix_output = self.range_mid
        self.steady_x_set = 73  # matches old "steady_X_set"

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

    def move(self, step_input: int, speed: int, command: str):
        """Execute movement pattern with detailed logging."""
        logger.info(f"\nMOVEMENT STEP {step_input}")
        logger.info(f"Speed: {speed} ({'-' if speed < 0 else '+'})")
        logger.info(f"Command: {command}")
        
        step_i = step_input
        step_ii = step_input + 2
        if step_ii > 4:
            step_ii = step_ii - 4
        
        if speed == 0:
            logger.info("Zero speed - no movement")
            return
        
        if command == 'no':
            logger.info("\nFIRST TRIPOD:")
            self.move_right_leg(1, step_i, speed)
            self.move_left_leg(2, step_i, speed)
            self.move_right_leg(3, step_i, speed)
            
            logger.info("\nSECOND TRIPOD:")
            self.move_left_leg(1, step_ii, speed)
            self.move_right_leg(2, step_ii, speed)
            self.move_left_leg(3, step_ii, speed)
        
        elif command == 'left':
            logger.info("\nFIRST TRIPOD WITH INVERTED SPEEDS:")
            self.move_right_leg(1, step_i, speed)
            self.move_left_leg(2, step_i, -speed)
            self.move_right_leg(3, step_i, speed)
            
            logger.info("\nSECOND TRIPOD WITH INVERTED SPEEDS:")
            self.move_left_leg(1, step_ii, -speed)
            self.move_right_leg(2, step_ii, speed)
            self.move_left_leg(3, step_ii, -speed)
        
        elif command == 'right':
            logger.info("\nFIRST TRIPOD WITH INVERTED SPEEDS:")
            self.move_right_leg(1, step_i, -speed)
            self.move_left_leg(2, step_i, speed)
            self.move_right_leg(3, step_i, -speed)
            
            logger.info("\nSECOND TRIPOD WITH INVERTED SPEEDS:")
            self.move_left_leg(1, step_ii, speed)
            self.move_right_leg(2, step_ii, -speed)
            self.move_left_leg(3, step_ii, speed)

    def move_left_leg(self, leg_num: int, pos: int, wiggle: int, height_adjust: int = 0):
        """Move a left leg with detailed logging of every servo movement."""
        servo_base = (leg_num - 1) * 2
        logger.info(f"\nLEFT LEG {leg_num} (servos {servo_base}=horizontal, {servo_base+1}=vertical):")
        logger.info(f"STEP {pos} with wiggle={wiggle}")
        logger.info(f"Current positions: horiz={self.pwm_values[servo_base]}, vert={self.pwm_values[servo_base+1]}")

        if pos == 0:
            if self.left_side_height:
                new_pwm = self.pwm_values[servo_base + 1] + height_adjust
                logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (height adjust)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)
            else:
                new_pwm = self.pwm_values[servo_base + 1] - height_adjust
                logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (height adjust)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)
            return

        if self.left_side_direction:
            if pos == 1:  # Lift leg
                logger.info("  STEP 1: Lifting leg")
                new_pwm = self.pwm_values[servo_base]
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (neutral)")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.left_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] + 3 * self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lift up)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] - 3 * self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lift up)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

            elif pos == 2:  # Move leg horizontally while lifted
                logger.info("  STEP 2: Moving leg horizontally while lifted")
                new_pwm = self.pwm_values[servo_base] + wiggle
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (move {wiggle:+})")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.left_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] - self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (partial lower)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] + self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (partial lower)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

            elif pos == 3:
                logger.info("  STEP 3: Leg back down, ignoring wiggle for vertical")
                new_pwm = self.pwm_values[servo_base]
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (neutral)")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.left_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] - self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] + self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

            elif pos == 4:
                logger.info("  STEP 4: Horizontal servo moves back (±wiggle)")
                new_pwm = self.pwm_values[servo_base] - wiggle
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (move {wiggle:-})")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.left_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] - self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] + self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

        else:
            if pos == 1:
                logger.info("  STEP 1: Lifting leg")
                new_pwm = self.pwm_values[servo_base]
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (neutral)")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.left_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] + 3 * wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lift up)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] - 3 * wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lift up)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

            elif pos == 2:
                logger.info("  STEP 2: Moving leg horizontally while lifted")
                new_pwm = self.pwm_values[servo_base] + wiggle
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (move {wiggle:+})")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.left_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] - wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (partial lower)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] + wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (partial lower)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

            elif pos == 3:
                logger.info("  STEP 3: Leg back down, ignoring wiggle for vertical")
                new_pwm = self.pwm_values[servo_base]
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (neutral)")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.left_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] - wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] + wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

            elif pos == 4:
                logger.info("  STEP 4: Horizontal servo moves back (±wiggle)")
                new_pwm = self.pwm_values[servo_base] + wiggle
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (move {wiggle:+})")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.left_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] - wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] + wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

    def move_right_leg(self, leg_num: int, pos: int, wiggle: int, height_adjust: int = 0):
        """Move a right leg with detailed logging of every servo movement."""
        servo_base = (leg_num - 1) * 2 + 6
        logger.info(f"RIGHT LEG {leg_num}:")
        logger.info(f"  Position: {pos}, Wiggle: {wiggle}, Height Adjust: {height_adjust}")
        logger.info(f"  Servo base: {servo_base} (horizontal) and {servo_base + 1} (vertical)")
        logger.info(f"  Current PWM values: horizontal={self.pwm_values[servo_base]}, vertical={self.pwm_values[servo_base + 1]}")
        
        if pos == 0:
            if self.right_side_height:
                new_pwm = self.pwm_values[servo_base + 1] + height_adjust
                logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (height adjust)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)
            else:
                new_pwm = self.pwm_values[servo_base + 1] - height_adjust
                logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (height adjust)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)
            return

        if self.right_side_direction:
            if pos == 1:
                logger.info("  STEP 1: Lifting leg")
                new_pwm = self.pwm_values[servo_base]
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (neutral)")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.right_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] + 3 * self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lift up)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] - 3 * self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lift up)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

            elif pos == 2:
                logger.info("  STEP 2: Moving leg horizontally while lifted")
                new_pwm = self.pwm_values[servo_base] + wiggle
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (move {wiggle:+})")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.right_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] - self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (partial lower)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] + self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (partial lower)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

            elif pos == 3:
                logger.info("  STEP 3: Leg back down, ignoring wiggle for vertical")
                new_pwm = self.pwm_values[servo_base]
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (neutral)")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.right_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] - self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] + self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

            elif pos == 4:
                logger.info("  STEP 4: Horizontal servo moves back (±wiggle)")
                new_pwm = self.pwm_values[servo_base] - wiggle
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (move {wiggle:-})")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.right_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] - self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] + self.height_change
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

        else:
            if pos == 1:
                logger.info("  STEP 1: Lifting leg")
                new_pwm = self.pwm_values[servo_base]
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (neutral)")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.right_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] + 3 * wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lift up)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] - 3 * wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lift up)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

            elif pos == 2:
                logger.info("  STEP 2: Moving leg horizontally while lifted")
                new_pwm = self.pwm_values[servo_base] + wiggle
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (move {wiggle:+})")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.right_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] - wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (partial lower)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] + wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (partial lower)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

            elif pos == 3:
                logger.info("  STEP 3: Leg back down, ignoring wiggle for vertical")
                new_pwm = self.pwm_values[servo_base]
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (neutral)")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.right_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] - wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] + wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

            elif pos == 4:
                logger.info("  STEP 4: Horizontal servo moves back (±wiggle)")
                new_pwm = self.pwm_values[servo_base] + wiggle
                logger.info(f"  Setting horizontal servo {servo_base} to {new_pwm} (move {wiggle:+})")
                self.sc.set_servo_pwm(servo_base, new_pwm)
                
                if self.right_side_height:
                    new_pwm = self.pwm_values[servo_base + 1] - wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                else:
                    new_pwm = self.pwm_values[servo_base + 1] + wiggle
                    logger.info(f"  Setting vertical servo {servo_base + 1} to {new_pwm} (lower)")
                self.sc.set_servo_pwm(servo_base + 1, new_pwm)

    def command(self, command_input: str):
        """Process movement commands exactly like old move.command()"""
        logger.info(f"Processing command: {command_input}")
        
        if command_input == 'forward':
            self.direction_command = 'forward'
            self.move_stu = 1
            self._thread_event.set()
            
        elif command_input == 'backward':
            self.direction_command = 'backward'
            self.move_stu = 1
            self._thread_event.set()
            
        elif command_input == 'left':
            self.turn_command = 'left'
            self.move_stu = 1
            self._thread_event.set()
            
        elif command_input == 'right':
            self.turn_command = 'right'
            self.move_stu = 1
            self._thread_event.set()
            
        elif command_input == 'stand':
            self.direction_command = 'stand'
            self.turn_command = 'no'  # Critical: Reset turn command
            self.move_stu = 0
            self._thread_event.clear()
            self.stand()
            self.step_set = 1
            
        elif command_input == 'no':
            self.turn_command = 'no'
            self.direction_command = 'no'  # Critical: Reset direction too
            self.move_stu = 0
            self._thread_event.clear()
            self.stand()

    def stand(self):
        """Make robot stand - CRITICAL: Set ALL servos to neutral"""
        logger.info("Standing - setting all servos to neutral")
        for i in range(16):  # Set ALL servos including camera
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
        """Main movement control loop - match old move_thread() exactly"""
        while True:
            self._thread_event.wait()
            
            if not self.steady_mode:
                if self.direction_command == 'forward' and self.turn_command == 'no':
                    if self.smooth_mode:
                        self.dove(self.step_set, self.dove_speed, 0.001, self.dpi, 'no')
                    else:
                        self.move(self.step_set, 35, 'no')
                        time.sleep(0.1)
                    
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1

                elif self.direction_command == 'backward' and self.turn_command == 'no':
                    if self.smooth_mode:
                        self.dove(self.step_set, -self.dove_speed, 0.001, self.dpi, 'no')
                    else:
                        self.move(self.step_set, -35, 'no')
                        time.sleep(0.1)
                    
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1

                elif self.turn_command != 'no':
                    if self.smooth_mode:
                        self.dove(self.step_set, 35, 0.001, self.dpi, self.turn_command)
                    else:
                        self.move(self.step_set, 35, self.turn_command)
                        time.sleep(0.1)
                    
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1

                elif self.direction_command == 'stand':
                    self.stand()
                    self.step_set = 1
            else:
                self._steady_x()
                self.steady()
            
            time.sleep(0.02)  # Prevent CPU overuse

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

    def dove(self, step_input: int, speed: int, time_last: float, dpi: int, command: str = 'no'):
        """Execute smooth movement using dove algorithm."""
        logger.info(f"\n{'='*50}")
        logger.info(f"DOVE MOVEMENT - Step {step_input}, Speed {speed}, Command {command}")
        logger.info(f"{'='*50}")
        
        if speed == 0:
            return

        # For backward movement (negative speed), we still use positive increments
        abs_speed = abs(speed)
        step_increment = int(abs_speed / dpi)
        
        for speed_i in range(0, abs_speed + step_increment, step_increment):
            speed_ii = speed_i
            speed_i = abs_speed - speed_i
            
            # If moving backward, invert horizontal movement only
            if speed < 0:
                horizontal = speed_i  # Will be negative for backward
            else:
                horizontal = -speed_i  # Normal forward movement
            
            # Vertical movement should always be the same regardless of direction
            vertical = 3 * self.height_change  # Use fixed height_change instead of speed_ii
            
            logger.info(f"\nDOVE CYCLE:")
            logger.info(f"speed_i={speed_i}, horizontal={horizontal}, vertical={vertical}")
            
            if step_input == 1:
                logger.info("\nSTEP 1 - Initial Lift")
                if command == 'no':
                    # First tripod - lift and move
                    logger.info("FIRST TRIPOD:")
                    self._dove_left_leg(0, horizontal, vertical)   # Left I
                    self._dove_right_leg(1, horizontal, vertical)  # Right II 
                    self._dove_left_leg(2, horizontal, vertical)   # Left III

                    # Second tripod - stay down
                    logger.info("SECOND TRIPOD:")
                    self._dove_right_leg(0, -horizontal, -10)  # Right I
                    self._dove_left_leg(1, -horizontal, -10)   # Left II
                    self._dove_right_leg(2, -horizontal, -10)  # Right III
                
                time.sleep(time_last / dpi)

    def _dove_left_leg(self, leg_num: int, horizontal: int, vertical: int):
        """Move a left leg in dove movement."""
        servo_base = leg_num * 2
        logger.info(f"\nDOVE LEFT LEG {leg_num+1}:")
        logger.info(f"Horizontal servo {servo_base}: {self.pwm_values[servo_base]} -> {self.pwm_values[servo_base] + horizontal}")
        logger.info(f"Vertical servo {servo_base+1}: {self.pwm_values[servo_base+1]} -> {self.pwm_values[servo_base+1] + vertical}")
        
        # Horizontal movement - affected by direction
        if self.left_side_direction:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] + horizontal)
        else:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] - horizontal)

        # Vertical movement - always use height_change
        if self.left_side_height:
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + vertical)
        else:
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] - vertical)

    def _dove_right_leg(self, leg_num: int, horizontal: int, vertical: int):
        """Move a right leg in dove movement."""
        servo_base = 6 + leg_num * 2
        logger.info(f"\nDOVE RIGHT LEG {leg_num+1}:")
        logger.info(f"Horizontal servo {servo_base}: {self.pwm_values[servo_base]} -> {self.pwm_values[servo_base] + horizontal}")
        logger.info(f"Vertical servo {servo_base+1}: {self.pwm_values[servo_base+1]} -> {self.pwm_values[servo_base+1] + vertical}")
        
        if self.right_side_direction:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] + horizontal)
        else:
            self.sc.set_servo_pwm(servo_base, self.pwm_values[servo_base] - horizontal)

        if self.right_side_height:
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] + vertical)
        else:
            self.sc.set_servo_pwm(servo_base + 1, self.pwm_values[servo_base + 1] - vertical)

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


