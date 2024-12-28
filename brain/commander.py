import threading
import time
import logging
import json
from typing import Dict, Any, Optional, Union, Callable

from servo import move
from system import info, config

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class Commander:
    """
    Process commands received from the WebSocket.
    """
    def __init__(
            self,
            servo_legs,
            servo_camera_lr,
            servo_camera_ud,
            light_strip,
            camera
    ):
        self.direction_command = None
        self.turn_command = None

        self.servo_legs = servo_legs
        self.servo_camera_lr = servo_camera_lr
        self.servo_camera_ud = servo_camera_ud
        self.light_strip = light_strip
        self.camera = camera

        # Initialize robot movement controller
        self.movement = move.RobotMovement(self.servo_legs)
        self.movement.set_init_positions(self.servo_legs.init_positions)

        # TODO: Is this correct use?
        self.init_pwms = json.loads(json.dumps(servo_legs.init_positions))

        self.camera_moving_lr = False
        self.camera_moving_ud = False

    def process(self, data: str) -> Dict[str, Any]:
        """Process commands received from the WebSocket."""
        logger.info(f'Received data: {data}')
        
        try:
            data_json = json.loads(data)
            if not isinstance(data_json, dict):
                return {"status": "error", "message": "JSON data must be an object"}
            return self._process_command(data_json)
        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON command: {data}. Error: {str(e)}")
            return {"status": "error", "message": f"Invalid JSON command: {str(e)}"}
        except Exception as e:
            logger.error(f"Unexpected error processing command: {str(e)}")
            return {"status": "error", "message": "Internal server error"}

    def _process_command(self, task: dict) -> Dict[str, Any]:
        """Handle JSON-formatted commands."""
        logger.info(f"JSON task: {task}")

        if "command" not in task:
            return {"status": "error", "message": "Invalid JSON command"}

        command = task["command"]

        # Update command mappings
        commands_available = {
            # Movement commands
            "forward": lambda: self._handle_movement("forward"),
            "backward": lambda: self._handle_movement("backward"),
            "move_stop": lambda: self._handle_movement("stand"),
            "left": lambda: self._handle_turn("left"),
            "right": lambda: self._handle_turn("right"),
            "turn_stop": lambda: self._handle_turn("no"),
            
            # Camera movement commands
            "look_left": lambda: self._handle_camera_look("left"),
            "look_right": lambda: self._handle_camera_look("right"),
            "look_up": lambda: self._handle_camera_look("up"),
            "look_down": lambda: self._handle_camera_look("down"),
            "look_lr_stop": lambda: self._handle_camera_stop("lr"),
            "look_ud_stop": lambda: self._handle_camera_stop("ud"),
            "camera_home": self._handle_camera_home,
            
            # System commands
            "get_info": self._handle_get_info,
            
            # Camera mode commands
            "findColor": lambda: self._handle_camera_mode("findColor"),
            "motionGet": lambda: self._handle_camera_mode("motionGet"),
            "stopCV": lambda: self._handle_camera_mode("stopCV"),
            
            # Movement mode commands
            "KD": lambda: move.command("KD"),
            "automaticOff": lambda: move.command("automaticOff"),
            "automatic": lambda: move.command("automatic"),
            
            # Light commands
            "police": lambda: self._handle_light("police"),
            "policeOff": lambda: self._handle_light("policeOff"),
            
            # Servo calibration commands
            **{f"SiLeft{i}": lambda x=i: self._handle_servo_calibration("SiLeft", x) for i in range(16)},
            **{f"SiRight{i}": lambda x=i: self._handle_servo_calibration("SiRight", x) for i in range(16)},
            **{f"PWMMS{i}": lambda x=i: self._handle_servo_calibration("PWMMS", x) for i in range(16)},
            "PWMINIT": lambda: self._handle_servo_calibration("PWMINIT"),
            "PWMD": lambda: self._handle_servo_calibration("PWMD"),

            # Autonomous behavior commands
            "startAutonomous": self.start_autonomous_behavior,
            "stopAutonomous": self.stop_autonomous_behavior,
        }
        
        # Execute command if it exists in mapping
        if command not in commands_available:
            return {"status": "error", "message": f"Command {command} not found"}
        
        try:
            result = commands_available[command]()
            return {"status": "ok", "data": result} if result is not None else {"status": "ok"}
        except Exception as e:
            logger.error(f"Error executing command {command}: {str(e)}")
            return {"status": "error", "message": f"Command execution failed: {str(e)}"}

    def _handle_movement(self, direction: str):
        """Handle movement commands (forward, backward, stand)."""
        valid_directions = {"forward", "backward", "stand"}
        if direction not in valid_directions:
            raise ValueError(f"Invalid movement direction: {direction}")
        
        self.direction_command = direction
        self.movement.command(self.direction_command)

    def _handle_turn(self, direction: str):
        """Handle turning commands (left, right, no)."""
        self.turn_command = "no" if direction == "turn_stop" else direction
        self.movement.command(self.turn_command)

    def _handle_camera_look(self, direction: str):
        """Handle camera look commands."""
        servo_map = {
            "left": (self.servo_camera_lr, 12, 1, "lr"),
            "right": (self.servo_camera_lr, 12, -1, "lr"),
            "up": (self.servo_camera_ud, 13, 1, "ud"),
            "down": (self.servo_camera_ud, 13, -1, "ud")
        }
        
        if direction not in servo_map:
            raise ValueError(f"Invalid camera direction: {direction}")
        
        servo, pin, value, axis = servo_map[direction]
        
        if axis == "lr":
            self.camera_moving_lr = True
        else:
            self.camera_moving_ud = True
        
        servo.single_servo(pin, value, 7)

    def _handle_camera_stop(self, axis: str):
        """Handle camera stop commands."""
        if axis == "lr":
            self.servo_camera_lr.stop_wiggle()
        elif axis == "ud":
            self.servo_camera_ud.stop_wiggle()

    def _handle_camera_home(self):
        """Reset camera position to home."""
        self.servo_camera_lr.move_angle(12, 0)
        self.servo_camera_ud.move_angle(13, 0)

    def _handle_get_info(self):
        """Get system information."""
        return {
            "title": "get_info",
            "data": [info.get_cpu_temp(), info.get_cpu_use(), info.get_ram_info()]
        }

    def _handle_camera_mode(self, mode: str):
        """Handle camera mode changes."""
        mode_map = {
            "findColor": "findColor",
            "motionGet": "watchDog",
            "stopCV": "none"
        }
        self.camera.modeSelect = mode_map[mode]

    def _handle_light(self, command: str):
        """Handle light strip commands."""
        if command == "police":
            self.light_strip.police()
        elif command == "policeOff":
            self.light_strip.pause()

    def _handle_servo_calibration(self, command: str, servo_num: Optional[int] = None) -> None:
        """Handle servo calibration commands."""
        if servo_num is not None and not (0 <= servo_num < 16):
            raise ValueError(f"Invalid servo number: {servo_num}")

        if command.startswith("Si"):
            adjustment = -1 if command == "SiLeft" else 1
            self.init_pwms[servo_num] = self.init_pwms[servo_num] + adjustment
            self.servo_legs.set_init_position(servo_num, self.init_pwms[servo_num], True)
        elif command == "PWMMS":
            config.write("pwm", f"init_pwm{servo_num}", self.init_pwms[servo_num])
        elif command == "PWMINIT":
            for i in range(0, 16):
                self.servo_legs.set_init_position(i, self.init_pwms[i], True)
        elif command == "PWMD":
            reset_pwm = {f"init_pwm{i}": 300 for i in range(0, 16)}
            config.write("pwm", None, reset_pwm)

    def start_autonomous_behavior(self):
        """Start autonomous robot behavior."""
        self.autonomous_thread = threading.Thread(target=self._autonomous_loop)
        self.autonomous_running = True
        self.autonomous_thread.start()

    def stop_autonomous_behavior(self):
        """Stop autonomous robot behavior."""
        self.autonomous_running = False
        if hasattr(self, 'autonomous_thread'):
            self.autonomous_thread.join()

    def _autonomous_loop(self):
        """Main autonomous behavior loop."""
        while self.autonomous_running:
            try:
                # Get camera frame and process it
                frame = self.camera.frame
                if frame is not None:
                    # Example: Look for colored objects
                    self.camera.modeSelect = "findColor"
                    
                    # If object detected, move towards it
                    if self.camera.findColorDetection:
                        self._handle_movement("forward")
                    else:
                        # Search pattern when no object detected
                        self._handle_movement("stand")
                        self._handle_camera_look("left")
                        time.sleep(2)
                        self._handle_camera_look("right")
                        time.sleep(2)
                
                # Add obstacle avoidance using MPU6050 sensor data
                if hasattr(self, 'sensor'):
                    accel_data = self.sensor.get_accel_data()
                    # Implement obstacle avoidance logic
                    
                time.sleep(0.1)  # Control loop rate
                
            except Exception as e:
                logger.error(f"Error in autonomous behavior: {e}")
                time.sleep(1)

    

    """
    # Single LEDs off (not used for now)
    # switch.switch(1,0)
    # switch.switch(2,0)
    # switch.switch(3,0)
    
    UNKNOWN COMMANDS
    # Depending on the received data, call the respective functions as before
    if isinstance(data, str):
        if 'wsB' in data:
            try:
                set_b = data.split()
                speed_set = int(set_b[1])
            except:
                pass

        elif 'AR' == data:
            modeSelect = 'AR'
            # What is this for?
            # screen.screen_show(4, 'ARM MODE ON')

        elif 'PT' == data:
            modeSelect = 'PT'
            # What is this for?
            # screen.screen_show(4, 'PT MODE ON')

        elif 'CVFLL1' in data:
            pos = int(data.split()[1])
            flask_app.camera.linePosSet_1(pos)

        elif 'CVFLL2' in data:
            pos = int(data.split()[1])
            flask_app.camera.linePosSet_2(pos)

        elif 'CVFLSP' in data:
            err = int(data.split()[1])
            flask_app.camera.errorSet(err)

        # elif 'defEC' in data:
        # 	fpv.defaultExpCom()

    """

    """
    # Single LEDs commands
    if 'Switch_1_on' in command_input:
        switch.switch(1,1)
    
    elif 'Switch_1_off' in command_input:
        switch.switch(1,0)
    
    elif 'Switch_2_on' in command_input:
        switch.switch(2,1)
    
    elif 'Switch_2_off' in command_input:
        switch.switch(2,0)
    
    elif 'Switch_3_on' in command_input:
        switch.switch(3,1)
    
    elif 'Switch_3_off' in command_input:
        switch.switch(3,0)
    """
