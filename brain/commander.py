import threading
import time
import logging
import json
from typing import Dict, Any, Optional, Union, Callable, List

from servo import legs, camera, base
from system import info, config
from servo.legs import LegsMovement
from servo.camera import CameraMovement

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class Commander:
    """
    Process commands received from the WebSocket.
    """
    def __init__(
            self,
            servo_legs,
            servo_camera,
            light_strip,
            camera
    ):
        self.direction_command = None
        self.turn_command = None

        self.servo_legs = servo_legs
        self.servo_camera = servo_camera
        self.light_strip = light_strip
        self.camera = camera

        # TODO: Is this correct use?
        self.init_pwms = json.loads(json.dumps(servo_legs.init_positions))

        self.legs_movement = LegsMovement(servo_legs)
        self.camera_movement = CameraMovement(servo_camera)

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
            "KD": lambda: self.legs_movement.command("KD"),
            "automaticOff": lambda: self.legs_movement.command("automaticOff"),
            "automatic": lambda: self.legs_movement.command("automatic"),
            
            # Light commands
            "police": lambda: self._handle_light("police"),
            "off": lambda: self._handle_light("off"),
            "stars": lambda: self._handle_light("stars"),
            "rainbow": lambda: self._handle_light("rainbow"),
            "breath": lambda: self._handle_light_breath(task),

            # Servo calibration commands
            "servo_set": lambda: self._handle_servo_calibration_set(task),
            "servo_save": lambda: self._handle_servo_calibration_save(task),
            "servo_center": lambda: self._handle_servo_calibration_center(task),
            "servo_reset": lambda: self._handle_servo_calibration_reset(task),

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
        """Handle movement commands (forward, backward, stand)"""
        valid_directions = {"forward", "backward", "stand"}
        if direction not in valid_directions:
            raise ValueError(f"Invalid movement direction: {direction}")
        
        # Always reset turn command when changing direction
        self.turn_command = "no"
        self.direction_command = direction
        self.legs_movement.command(self.direction_command)  # Use legs_movement instance

    def _handle_turn(self, direction: str):
        """Handle turning commands (left, right, no)"""
        # Always reset direction command when turning
        self.direction_command = "no"
        self.turn_command = "no" if direction == "turn_stop" else direction
        self.legs_movement.command(self.turn_command)  # Updated to use legs_movement instance

    def _handle_camera_look(self, direction: str) -> None:
        """Handle camera look commands."""
        logger.info(f"Camera look command: {direction}")
        self.camera_movement.move(direction, continuous=True)

    def _handle_camera_stop(self, axis: str) -> None:
        """Handle camera stop commands."""
        logger.info(f"Camera stop command: {axis}")
        self.camera_movement.stop(axis)

    def _handle_camera_home(self) -> None:
        """Handle camera home command."""
        logger.info("Camera home command")
        self.camera_movement.home()

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
        elif command == "off":
            self.light_strip.off()
        elif command == "stars":
            self.light_strip.stars()
        elif command == "rainbow":
            self.light_strip.rainbow()

    def _handle_light_breath(self, task: dict) -> None:
        """Handle breath mode with RGB values."""
        if not all(k in task for k in ["r", "g", "b"]):
            raise ValueError("Breath command requires r, g, b values")
        self.light_strip.breath(task["r"], task["g"], task["b"])

    def _get_servo_controller(self, servo_ids: List[int]) -> base.ServoCtrl:
        """
        Get the appropriate servo controller for the given servo IDs.
        Raises ValueError if servos are from different groups.
        """
        # Camera servo IDs are 12 and 13
        camera_servos = {12, 13}
        leg_servos = set(range(12))  # 0-11 are leg servos
        
        servo_set = set(servo_ids)
        if servo_set.issubset(camera_servos):
            return self.servo_camera
        elif servo_set.issubset(leg_servos):
            return self.servo_legs
        else:
            raise ValueError("Cannot mix leg and camera servos in the same command")

    def _handle_servo_calibration_set(self, task: dict) -> Dict[str, Any]:
        """Handle servo_set calibration command."""
        required_fields = {"servos", "direction", "steps"}
        if not all(field in task for field in required_fields):
            raise ValueError(f"Missing required fields. Need: {required_fields}")
            
        servos = task["servos"]
        direction = task["direction"]
        steps = task["steps"]
        
        if not isinstance(servos, list):
            raise ValueError("servos must be a list of servo IDs")
        if not servos:
            raise ValueError("servos list cannot be empty")
            
        servo_ctrl = self._get_servo_controller(servos)
        return servo_ctrl.adjust_servo_positions(servos, direction, steps)

    def _handle_servo_calibration_save(self, task: dict) -> Dict[str, Any]:
        """Handle servo_save calibration command."""
        if "servos" not in task:
            raise ValueError("Missing required field: servos")
            
        servos = task["servos"]
        if not isinstance(servos, list):
            raise ValueError("servos must be a list of servo IDs")
        if not servos:
            raise ValueError("servos list cannot be empty")
            
        servo_ctrl = self._get_servo_controller(servos)
        return servo_ctrl.save_current_positions(servos)

    def _handle_servo_calibration_center(self, task: dict) -> Dict[str, Any]:
        """Handle servo_center calibration command."""
        if "servos" not in task:
            raise ValueError("Missing required field: servos")
            
        servos = task["servos"]
        if not isinstance(servos, list):
            raise ValueError("servos must be a list of servo IDs")
        if not servos:
            raise ValueError("servos list cannot be empty")
            
        servo_ctrl = self._get_servo_controller(servos)
        return servo_ctrl.center_servos(servos)

    def _handle_servo_calibration_reset(self, task: dict) -> Dict[str, Any]:
        """Handle servo_reset calibration command."""
        if "servos" not in task:
            raise ValueError("Missing required field: servos")
            
        servos = task["servos"]
        if not isinstance(servos, list):
            raise ValueError("servos must be a list of servo IDs")
        if not servos:
            raise ValueError("servos list cannot be empty")
            
        servo_ctrl = self._get_servo_controller(servos)
        return servo_ctrl.reset_servos(servos)

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
                
                # # Add obstacle avoidance using MPU6050 sensor data
                # if hasattr(self, 'sensor'):
                #     accel_data = self.sensor.get_accel_data()
                #     # Implement obstacle avoidance logic
                    
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
