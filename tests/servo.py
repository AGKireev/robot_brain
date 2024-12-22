import os
import sys
import time
import busio
import logging
from typing import List
from adafruit_pca9685 import PCA9685
from adafruit_motor.servo import Servo
from board import SCL, SDA

# Add the parent directory of 'server' to sys.path
script_dir = os.path.realpath(os.path.dirname(__file__))
sys.path.append(os.path.abspath(os.path.join(script_dir, '..', 'server')))
from servo import base

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Initialize I2C and PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Default pulse width range for servos
MIN_PULSE_US = 488  # Minimum pulse width in microseconds
MAX_PULSE_US = 2538  # Maximum pulse width in microseconds

# Create servo objects for all channels
servos = [Servo(pca.channels[i], min_pulse=MIN_PULSE_US, max_pulse=MAX_PULSE_US) for i in range(16)]

def parse_servo_ids(ids: str) -> List[int]:
    """
    Parses a comma-separated string of servo IDs into a list of integers.
    """
    try:
        return [int(i.strip()) for i in ids.split(",") if i.strip().isdigit()]
    except ValueError:
        logger.error("Invalid servo ID format. Must be a comma-separated list of integers.")
        sys.exit(1)

def move_fast(servo_ids: List[int] | None = None) -> None:
    """
    Makes the specified servos oscillate between two positions quickly.
    """
    logger.info(f"Moving servos {servo_ids} quickly.")
    if servo_ids is None:
        servo_ids = [0]
    cycles = 10
    for _ in range(cycles):  # Repeat N cycles
        logger.info(f"Cycle {_ + 1} of {cycles}")
        for servo_id in servo_ids:
            logger.info(f"servo {servo_id} forward ...")
            servos[servo_id].angle = 90  # Move to 90 degrees
        time.sleep(1)

        for servo_id in servo_ids:
            logger.info(f"servo {servo_id} backward ...")
            servos[servo_id].angle = 120  # Move to 120 degrees
        time.sleep(1)

def move_slow(servo_ids: List[int] | None = None) -> None:
    """
    Makes the specified servos move slowly between two positions.
    """
    logger.info(f"Moving servos {servo_ids} slowly.")
    if servo_ids is None:
        servo_ids = [0]
    cycles = 10
    for _ in range(cycles):  # Repeat N cycles
        logger.info(f"Cycle {_ + 1} of {cycles}")
        for servo_id in servo_ids:
            logger.info(f"servo {servo_id} forward ...")
            for angle in range(90, 121):  # Slowly move from 90 to 120 degrees
                servos[servo_id].angle = angle
                time.sleep(0.05)

        for servo_id in servo_ids:
            logger.info(f"servo {servo_id} backward ...")
            for angle in range(120, 89, -1):  # Slowly move back from 120 to 90 degrees
                servos[servo_id].angle = angle
                time.sleep(0.05)

def use_library(servo_ids: List[int] | None = None) -> None:
    """
    Uses the project library to control the specified servos
    :param servo_ids:
    :return:
    """
    logger.info(f"Using library to control servos {servo_ids}.")
    controller = None
    try:
        controller = base.ServoCtrl()
        controller.start()
        cycles = 10
        alternate = True  # Flag to alternate positions
        for _ in range(cycles):
            logger.info(f"Cycle {_ + 1} of {cycles}")
            if alternate:
                controller.auto_speed(servo_ids, [45, -45])
            else:
                controller.auto_speed(servo_ids, [-45, 45])
            alternate = not alternate  # Toggle the alternate flag
            while controller.running.is_set():
                time.sleep(0.1)  # Wait for the controller to finish
            time.sleep(1)  # Pause before the next cycle
    except KeyboardInterrupt:
        logger.info("Interrupted while using the library.")
    finally:
        if controller:
            controller.shutdown()

def init_servos(servo_ids: List[int] | None = None) -> None:
    """
    Initializes specified servos to their default positions
    """
    if servo_ids is None:
        servo_ids = [0]
    logger.info("Initializing servos to default positions: %s", servo_ids)
    controller = None
    try:
        controller = base.ServoCtrl()
        controller.start()
        controller.move_init(servo_ids)
        time.sleep(2)  # Allow time for servos to move to default position
    except KeyboardInterrupt:
        logger.info("Interrupted during initialization.")
    finally:
        if controller:
            controller.shutdown()

def shutdown() -> None:
    """
    Gracefully shuts down the PCA9685 controller.
    """
    logger.info("Shutting down PCA9685 controller...")
    for servo in servos:
        servo.angle = None  # Disable all servos
    pca.deinit()

if __name__ == '__main__':
    logger.info("Starting servo test...")
    try:
        if len(sys.argv) > 1:
            command = sys.argv[1]
            servos_to_move = parse_servo_ids(sys.argv[2]) if len(sys.argv) > 2 else [0]

            if command == 'fast':
                move_fast(servos_to_move)
            elif command == 'slow':
                move_slow(servos_to_move)
            elif command == 'library':
                use_library(servos_to_move)
            elif command == 'init':
                init_servos(servos_to_move)
            else:
                logger.error('Invalid command. Usage: python servo.py [fast|slow|library|init] [servo_ids]')
                sys.exit(1)
        else:
            logger.error('No command provided. Usage: python servo.py [fast|slow|library|init] [servo_ids]')
            sys.exit(1)
    except KeyboardInterrupt:
        logger.info("Exiting...")
    finally:
        shutdown()
    logger.info("Servo test complete.")
