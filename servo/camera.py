import logging

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
        self.horizontal_min = 140
        self.vertical_max = 500
        self.vertical_min = 270
        logger.info(f"Movement limits set - Horizontal: [{self.horizontal_min}, {self.horizontal_max}], Vertical: [{self.vertical_min}, {self.vertical_max}]")
        
        # Get channel numbers from servo controller
        self.horizontal_channel = self.sc.pwm_channels[0]  # Channel 12
        self.vertical_channel = self.sc.pwm_channels[1]    # Channel 13
        logger.info(f"Camera channels - Horizontal: {self.horizontal_channel}, Vertical: {self.vertical_channel}")
        
        # Current positions
        self._horizontal_pos = self.sc.init_positions[self.horizontal_channel]
        self._vertical_pos = self.sc.init_positions[self.vertical_channel]
        logger.info(f"Initial positions - Horizontal: {self._horizontal_pos}, Vertical: {self._vertical_pos}")

    def _can_move(self, direction: str) -> bool:
        """Check if movement in given direction is possible."""
        if direction == 'left' and self._horizontal_pos >= self.horizontal_max:
            logger.info(f"Cannot move left - Already at maximum horizontal position ({self._horizontal_pos})")
            return False
        elif direction == 'right' and self._horizontal_pos <= self.horizontal_min:
            logger.info(f"Cannot move right - Already at minimum horizontal position ({self._horizontal_pos})")
            return False
        elif direction == 'up' and self._vertical_pos >= self.vertical_max:
            logger.info(f"Cannot move up - Already at maximum vertical position ({self._vertical_pos})")
            return False
        elif direction == 'down' and self._vertical_pos <= self.vertical_min:
            logger.info(f"Cannot move down - Already at minimum vertical position ({self._vertical_pos})")
            return False
        return True

    def move(self, direction: str, continuous: bool = True, speed: int = 7) -> None:
        """Move camera in specified direction.
        
        Args:
            direction: One of 'left', 'right', 'up', 'down'
            continuous: If True, move continuously until stopped. If False, move by one step.
            speed: Movement speed (1-10, default: 7)
        """
        if direction not in ('left', 'right', 'up', 'down'):
            logger.error(f"Invalid direction specified: {direction}")
            raise ValueError(f"Invalid camera direction: {direction}")
            
        if not self._can_move(direction):
            return
            
        logger.info(f"Moving camera {direction} - Mode: {'continuous' if continuous else 'single step'}, Speed: {speed}")
        
        # Map directions to channels and values
        direction_map = {
            'left': (self.horizontal_channel, 1),
            'right': (self.horizontal_channel, -1),
            'up': (self.vertical_channel, 1),
            'down': (self.vertical_channel, -1)
        }
        
        channel, value = direction_map[direction]
        
        if continuous:
            # For continuous movement, use single_servo which runs in a separate thread
            logger.debug(f"Starting continuous movement on channel {channel} with direction {value}")
            self.sc.single_servo(channel, value, speed)
        else:
            # For single step movement, calculate new position
            step = 30  # Default step size
            if direction in ('left', 'up'):
                new_pos = min(
                    self._get_current_pos(channel) + step,
                    self.horizontal_max if channel == self.horizontal_channel else self.vertical_max
                )
            else:  # right or down
                new_pos = max(
                    self._get_current_pos(channel) - step,
                    self.horizontal_min if channel == self.horizontal_channel else self.vertical_min
                )
            
            logger.debug(f"Moving to position {new_pos} on channel {channel}")
            self.sc.set_pwm(channel, new_pos)
            self._update_current_pos(channel, new_pos)

    def _get_current_pos(self, channel: int) -> int:
        """Get current position for given channel."""
        return self._horizontal_pos if channel == self.horizontal_channel else self._vertical_pos

    def _update_current_pos(self, channel: int, pos: int) -> None:
        """Update current position tracking."""
        if channel == self.horizontal_channel:
            self._horizontal_pos = pos
        else:
            self._vertical_pos = pos

    def stop(self, axis: str) -> None:
        """Stop camera movement on specified axis.
        
        Args:
            axis: Either 'lr' (left-right) or 'ud' (up-down)
        """
        logger.info(f"Stopping movement on {axis} axis")
        if axis not in ('lr', 'ud'):
            logger.error(f"Invalid axis specified: {axis}")
            raise ValueError(f"Invalid axis: {axis}. Must be 'lr' or 'ud'")
        self.sc.stop_wiggle()
        logger.info(f"Movement stopped on {axis} axis")

    def home(self) -> None:
        """Reset camera to home position."""
        logger.info("Moving camera to home position")
        logger.debug(f"Current positions - Horizontal: {self._horizontal_pos}, Vertical: {self._vertical_pos}")
        
        # Move horizontal
        new_h = self.sc.init_positions[self.horizontal_channel]
        self.sc.set_pwm(self.horizontal_channel, new_h)
        logger.info(f"Horizontal position changed: {self._horizontal_pos} -> {new_h}")
        self._horizontal_pos = new_h
        
        # Move vertical
        new_v = self.sc.init_positions[self.vertical_channel]
        self.sc.set_pwm(self.vertical_channel, new_v)
        logger.info(f"Vertical position changed: {self._vertical_pos} -> {new_v}")
        self._vertical_pos = new_v