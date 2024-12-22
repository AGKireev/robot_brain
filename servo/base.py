# ======================================================================
# IMPORTANT UPDATE NOTES:
#
# This code was originally written using the legacy 'Adafruit_PCA9685' library
# and 'pwm.set_pwm(...)' calls to control servo position with raw PWM values.
# We are now using the newer Adafruit CircuitPython libraries:
#   - adafruit_pca9685
#   - adafruit_motor.servo
#
# The original code controlled servo positions by sending values like 100 to 520
# to pwm.set_pwm(...). Those values corresponded to a certain pulse width that mapped
# linearly to angles from 0 to 180 degrees. We must preserve the exact same functionality,
# behavior, speed, and range to avoid damaging the devices and maintain external script compatibility.
#
# Original key parameters:
#   ctrlRangeMin = 100 steps
#   ctrlRangeMax = 520 steps
#   angleRange   = 180 degrees
#
# This means:
#   100 steps -> 0 degrees
#   520 steps -> 180 degrees
#
# We will keep all internal arrays (init_positions, goalPos, etc.) and computations in terms of these "PWM step" values.
# Only at the point of actually commanding the servo, we will convert these step values to an angle for the servo library.
#
# Conversion from steps to angle:
#   angle = ((PWM_steps - ctrlRangeMin) / (ctrlRangeMax - ctrlRangeMin)) * angleRange
#   angle = ((steps - 100) / (420)) * 180
#
# Since we are using adafruit_motor.servo, we specify min_pulse and max_pulse so that:
#   angle=0° -> ~488us
#   angle=180° -> ~2538us
# This matches original scaling. Each "step" approx. 4.88us, so 100 steps ~488us and 520 steps ~2538us.
# ======================================================================
import time
import threading
import logging
from typing import List, Union, Optional
from board import SCL, SDA
import busio
import adafruit_pca9685
from adafruit_motor import servo

from system import config

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize I2C and PCA9685
i2c = busio.I2C(SCL, SDA)
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 50

# Define pulse width range in microseconds for 0 to 180 degrees
MIN_PULSE_US = 500  # 488
MAX_PULSE_US = 2500 # 2538

# Load initial servo positions from config
pwm_config = config.read("pwm")
init_positions = [pwm_config[f"init_pwm{i}"] for i in range(16)]

# Create servo instances for all 16 channels
servos = [servo.Servo(pca.channels[i], min_pulse=MIN_PULSE_US, max_pulse=MAX_PULSE_US) for i in range(16)]

class ServoCtrl(threading.Thread):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        # if the servo rotates inversely,
        # just change the N-th number in the array to -1
        # to reverse the direction
        self.sc_direction: List[int] = [1] * 16
        self.init_positions: List[int] = init_positions
        self.goal_positions: List[int] = [300] * 16
        self.current_positions: List[int] = [300] * 16
        self.buffer_positions: List[float] = [300.0] * 16
        self.last_positions: List[int] = [300] * 16
        self.ing_goal: List[int] = [300] * 16
        self.min_positions: List[int] = [100] * 16
        self.max_positions: List[int] = [520] * 16
        self.sc_speed: List[int] = [0] * 16

        self.ctrl_range_max: int = 520
        self.ctrl_range_min: int = 100
        self.angle_range: int = 180

        '''
        scMode: 'init' 'auto' 'certain' 'quick' 'wiggle'
        '''
        self.sc_mode: str = 'auto'
        self.sc_time: float = 2.0
        self.sc_steps: int = 30
        self.sc_delay: float = 0.037
        self.sc_move_time: float = 0.037

        self.goal_update: int = 0
        self.wiggle_id: int = 0
        self.wiggle_direction: int = 1

        self.running = threading.Event()
        self.running.clear()

    def pause(self) -> None:
        logger.info("ServoCtrl: pause")
        self.running.clear()

    def resume(self) -> None:
        logger.info("ServoCtrl: resume")
        self.running.set()

    def pwm_to_angle(self, pwm: int) -> int:
        return int(max(0, min(int((pwm - self.ctrl_range_min) / (self.ctrl_range_max - self.ctrl_range_min) * self.angle_range), 180)))

    def set_servo_pwm(self, channel: int, pwm: int) -> None:
        if self.min_positions[channel] <= pwm <= self.max_positions[channel]:
            angle = self.pwm_to_angle(pwm)
            servos[channel].angle = angle
            self.current_positions[channel] = pwm
        else:
            logger.warning(f"PWM value {pwm} out of range for channel {channel}.")

    def move_init(self, ids: Union[List[int], int, None] = None) -> None:
        """
        Initialize servos.
        - ids=None: Initializes all servos.
        - ids=list: Initializes specific servos.
        - ids=int: Initializes a single servo.
        """
        if ids is None:  # Initialize all servos
            ids = range(16)
        elif isinstance(ids, int):  # Single servo
            ids = [ids]

        for i in ids:
            self.set_servo_pwm(i, self.init_positions[i])
            self.last_positions[i] = self.init_positions[i]
            self.current_positions[i] = self.init_positions[i]
            self.buffer_positions[i] = float(self.init_positions[i])
            self.goal_positions[i] = self.init_positions[i]

        self.sc_mode = 'init'
        self.pause()

    def set_init_position(self, id: int, init_input: int, move_to: bool = False) -> None:
        """
        Updates the initial position of a specific servo.

        Args:
            id (int): The servo ID.
            init_input (int): The new initial position (PWM value).
            move_to (bool): If True, moves the servo to the updated initial position immediately.
        """
        if self.min_positions[id] <= init_input <= self.max_positions[id]:
            self.init_positions[id] = init_input
            if move_to:
                self.set_servo_pwm(id, init_input)
        else:
            logger.error(f"Invalid initial position {init_input} for servo {id}.")

    def pos_update(self) -> None:
        self.goal_update = 1
        for i in range(16):
            self.last_positions[i] = self.current_positions[i]
        self.goal_update = 0

    def speed_update(self, ids: List[int], speeds: List[int]) -> None:
        for i, speed in zip(ids, speeds):
            self.sc_speed[i] = speed

    def move_auto(self) -> None:
        for i in range(16):
            self.ing_goal[i] = self.goal_positions[i]

        for step in range(self.sc_steps):
            for channel in range(16):
                if not self.goal_update:
                    delta = (self.goal_positions[channel] - self.last_positions[channel]) / self.sc_steps
                    self.current_positions[channel] = int(round(self.last_positions[channel] + delta * (step + 1)))
                    self.set_servo_pwm(channel, self.current_positions[channel])
            time.sleep(self.sc_time / self.sc_steps)
        self.pos_update()
        self.pause()

    def move_cert(self) -> None:
        for i in range(16):
            self.ing_goal[i] = self.goal_positions[i]
            self.buffer_positions[i] = self.last_positions[i]

        while self.current_positions != self.goal_positions:
            for i in range(16):
                if self.last_positions[i] < self.goal_positions[i]:
                    self.buffer_positions[i] += self.sc_speed[i] / (1 / self.sc_delay)
                elif self.last_positions[i] > self.goal_positions[i]:
                    self.buffer_positions[i] -= self.sc_speed[i] / (1 / self.sc_delay)
                self.current_positions[i] = int(round(self.buffer_positions[i]))
                self.set_servo_pwm(i, self.current_positions[i])
            time.sleep(self.sc_delay - self.sc_move_time)
        self.pos_update()
        self.pause()

    def pwm_gen_out(self, angle: float) -> int:
        return int(round((self.ctrl_range_max - self.ctrl_range_min) / self.angle_range * angle, 0))

    def set_auto_time(self, time: float) -> None:
        self.sc_time = time

    def set_delay(self, delay: float) -> None:
        self.sc_delay = delay

    def auto_speed(self, ids: List[int], angles: List[float]) -> None:
        self.sc_mode = 'auto'
        self.goal_update = 1
        for i, angle in zip(ids, angles):
            target = self.init_positions[i] + self.pwm_gen_out(angle) * self.sc_direction[i]
            self.goal_positions[i] = max(self.min_positions[i], min(target, self.max_positions[i]))
        self.goal_update = 0
        self.resume()

    def cert_speed(self, ids: List[int], angles: List[float], speeds: List[int]) -> None:
        self.sc_mode = 'certain'
        self.goal_update = 1
        for i, angle in zip(ids, angles):
            target = self.init_positions[i] + self.pwm_gen_out(angle) * self.sc_direction[i]
            self.goal_positions[i] = max(self.min_positions[i], min(target, self.max_positions[i]))
        self.speed_update(ids, speeds)
        self.goal_update = 0
        self.resume()

    def move_wiggle(self) -> None:
        while self.running.is_set():
            delta = self.wiggle_direction * self.sc_speed[self.wiggle_id] / (1 / self.sc_delay)
            self.buffer_positions[self.wiggle_id] += delta * self.sc_direction[self.wiggle_id]
            self.current_positions[self.wiggle_id] = int(round(self.buffer_positions[self.wiggle_id]))
            self.set_servo_pwm(self.wiggle_id, self.current_positions[self.wiggle_id])
            time.sleep(self.sc_delay - self.sc_move_time)

    def stop_wiggle(self) -> None:
        self.pause()
        self.pos_update()

    def single_servo(self, id: int, direction: int, speed: int) -> None:
        self.wiggle_id = id
        self.wiggle_direction = direction
        self.sc_speed[id] = speed
        self.sc_mode = 'wiggle'
        self.resume()

    def move_angle(self, id: int, angle: float) -> None:
        pwm = self.init_positions[id] + self.pwm_gen_out(angle) * self.sc_direction[id]
        self.current_positions[id] = max(self.min_positions[id], min(int(pwm), self.max_positions[id]))
        self.last_positions[id] = self.current_positions[id]
        self.set_servo_pwm(id, self.current_positions[id])

    def sc_move(self) -> None:
        if self.sc_mode == 'init':
            self.move_init()
        elif self.sc_mode == 'auto':
            self.move_auto()
        elif self.sc_mode == 'certain':
            self.move_cert()
        elif self.sc_mode == 'wiggle':
            self.move_wiggle()

    def set_pwm(self, id: int, pwm: int) -> None:
        self.last_positions[id] = pwm
        self.current_positions[id] = pwm
        self.buffer_positions[id] = float(pwm)
        self.goal_positions[id] = pwm
        self.set_servo_pwm(id, pwm)
        self.pause()

    def shutdown(self) -> None:
        """
        Gracefully shut down the servo controller.
        """
        logger.info("Shutting down ServoCtrl...")
        self.pause()
        for s in servos:
            s.angle = None  # Disable all servos
        pca.deinit()
        logger.info("ServoCtrl shut down successfully.")

    def run(self) -> None:
        while True:
            self.running.wait()
            self.sc_move()
