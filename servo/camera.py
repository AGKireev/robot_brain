import logging
from typing import Optional

from servo import base

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class CameraMovement:
    def __init__(self, servo_ctrl: base.ServoCtrl):
        """Initialize camera movement controller.
        
        Args:
            servo_ctrl: ServoCtrl instance configured for camera servos
        """
        self.sc = servo_ctrl
        logger.info("Initializing CameraMovement with servo controller")
        
        # Movement limits
        self.horizontal_max = 500
        self.horizontal_min = 100
        self.vertical_max = 500
        self.vertical_min = 270
        logger.info(f"Movement limits set - Horizontal: [{self.horizontal_min}, {self.horizontal_max}], Vertical: [{self.vertical_min}, {self.vertical_max}]")
        
        # Default movement step
        self.look_wiggle = 30
        logger.info(f"Default movement step (wiggle) set to: {self.look_wiggle}")
        
        # Get channel numbers from servo controller
        self.horizontal_channel = self.sc.pwm_channels[0]  # Channel 12
        self.vertical_channel = self.sc.pwm_channels[1]    # Channel 13
        logger.info(f"Camera channels - Horizontal: {self.horizontal_channel}, Vertical: {self.vertical_channel}")
        
        # Current positions
        self._horizontal_pos = self.sc.init_positions[self.horizontal_channel]
        self._vertical_pos = self.sc.init_positions[self.vertical_channel]
        logger.info(f"Initial positions - Horizontal: {self._horizontal_pos}, Vertical: {self._vertical_pos}")

    def _update_position(self, channel: int, value: int, min_val: int, max_val: int) -> int:
        """Update servo position within limits."""
        logger.debug(f"Updating position - Channel: {channel}, Requested value: {value}, Range: [{min_val}, {max_val}]")
        new_pos = max(min_val, min(value, max_val))
        if new_pos != value:
            logger.info(f"Position value clamped from {value} to {new_pos} for channel {channel}")
        self.sc.set_pwm(channel, new_pos)
        logger.debug(f"Position updated - Channel: {channel}, New position: {new_pos}")
        return new_pos

    def look_up(self, wiggle: Optional[int] = None) -> None:
        """Move camera up."""
        wiggle = self.look_wiggle if wiggle is None else wiggle
        logger.info(f"Looking up - Current vertical pos: {self._vertical_pos}, Wiggle: {wiggle}")
        
        # Direction is handled by ServoCtrl's direction configuration
        old_pos = self._vertical_pos
        self._vertical_pos = self._update_position(
            self.vertical_channel,
            self._vertical_pos + wiggle,
            self.vertical_min,
            self.vertical_max
        )
        logger.info(f"Vertical position changed: {old_pos} -> {self._vertical_pos}")

    def look_down(self, wiggle: Optional[int] = None) -> None:
        """Move camera down."""
        wiggle = self.look_wiggle if wiggle is None else wiggle
        logger.info(f"Looking down - Current vertical pos: {self._vertical_pos}, Wiggle: {wiggle}")
        
        old_pos = self._vertical_pos
        self._vertical_pos = self._update_position(
            self.vertical_channel,
            self._vertical_pos - wiggle,
            self.vertical_min,
            self.vertical_max
        )
        logger.info(f"Vertical position changed: {old_pos} -> {self._vertical_pos}")

    def look_left(self, wiggle: Optional[int] = None) -> None:
        """Move camera left."""
        wiggle = self.look_wiggle if wiggle is None else wiggle
        logger.info(f"Looking left - Current horizontal pos: {self._horizontal_pos}, Wiggle: {wiggle}")
        
        old_pos = self._horizontal_pos
        self._horizontal_pos = self._update_position(
            self.horizontal_channel,
            self._horizontal_pos + wiggle,
            self.horizontal_min,
            self.horizontal_max
        )
        logger.info(f"Horizontal position changed: {old_pos} -> {self._horizontal_pos}")

    def look_right(self, wiggle: Optional[int] = None) -> None:
        """Move camera right."""
        wiggle = self.look_wiggle if wiggle is None else wiggle
        logger.info(f"Looking right - Current horizontal pos: {self._horizontal_pos}, Wiggle: {wiggle}")
        
        old_pos = self._horizontal_pos
        self._horizontal_pos = self._update_position(
            self.horizontal_channel,
            self._horizontal_pos - wiggle,
            self.horizontal_min,
            self.horizontal_max
        )
        logger.info(f"Horizontal position changed: {old_pos} -> {self._horizontal_pos}")

    def look_home(self) -> None:
        """Reset camera to home position."""
        logger.info("Moving camera to home position")
        logger.debug(f"Current positions - Horizontal: {self._horizontal_pos}, Vertical: {self._vertical_pos}")
        
        old_h = self._horizontal_pos
        old_v = self._vertical_pos
        
        self._horizontal_pos = self._update_position(
            self.horizontal_channel,
            self.sc.init_positions[self.horizontal_channel],
            self.horizontal_min,
            self.horizontal_max
        )
        self._vertical_pos = self._update_position(
            self.vertical_channel,
            self.sc.init_positions[self.vertical_channel],
            self.vertical_min,
            self.vertical_max
        )
        logger.info(f"Position changed - Horizontal: {old_h} -> {self._horizontal_pos}, Vertical: {old_v} -> {self._vertical_pos}")

    def stop_movement(self, axis: str) -> None:
        """Stop camera movement on specified axis."""
        logger.info(f"Stopping movement on axis: {axis}")
        if axis not in ('lr', 'ud'):
            logger.error(f"Invalid axis specified: {axis}")
            raise ValueError(f"Invalid axis: {axis}. Must be 'lr' or 'ud'")
        self.sc.stop_wiggle()
        logger.info(f"Movement stopped on {axis} axis")

    def single_servo(self, direction: str, speed: int = 7) -> None:
        """Control a single servo movement.
        
        Args:
            direction: One of 'left', 'right', 'up', 'down'
            speed: Movement speed (default: 7)
        """
        logger.info(f"Single servo movement - Direction: {direction}, Speed: {speed}")
        servo_map = {
            'left': (self.horizontal_channel, 1),
            'right': (self.horizontal_channel, -1),
            'up': (self.vertical_channel, 1),
            'down': (self.vertical_channel, -1)
        }
        
        if direction not in servo_map:
            logger.error(f"Invalid direction specified: {direction}")
            raise ValueError(f"Invalid camera direction: {direction}")
        
        channel, value = servo_map[direction]
        logger.debug(f"Using channel {channel} with direction value {value}")
        self.sc.single_servo(channel, value, speed)
        logger.info(f"Single servo movement started - Channel: {channel}, Direction: {value}, Speed: {speed}")