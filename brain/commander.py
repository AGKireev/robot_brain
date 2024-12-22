import logging

from servo import move
from system import info, config

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class Commander:
    """
    Process commands received from the WebSocket.
    """
    def __init__(self, legs, P_sc, T_sc, RL, camera):  # TODO: rename
        self.direction_command = None
        self.turn_command = None

        self.legs = legs
        self.P_sc = P_sc
        self.T_sc = T_sc
        self.RL = RL
        self.camera = camera

        self.init_pwms = legs.init_positions.copy()

    def process(self, command_input: str):
        """
        Process commands received from the WebSocket.
        """
        logger.info(f'Received data: {command_input}')

        response = {"status": "ok", "title": "", "data": None}

        # Movement commands
        if command_input == "forward":
            self.direction_command = command_input
            move.command(command_input)

        elif command_input == "backward":
            self.direction_command = command_input
            move.command(command_input)

        elif command_input == "DS":  # TODO: Rename to move_stop
            self.direction_command = "stand"
            move.command(self.direction_command)

        elif command_input == "left":
            self.turn_command = command_input
            move.command(command_input)

        elif command_input == "right":
            self.turn_command = command_input
            move.command(command_input)

        elif command_input == "TS":  # TODO: Rename to turn_stop
            self.turn_command = "no"
            move.command(self.turn_command)

        # Camera commands
        elif command_input == "lookleft":
            self.P_sc.single_servo(12, 1, 7)

        elif command_input == "lookright":
            self.P_sc.single_servo(12, -1, 7)

        elif command_input == "LRstop":  # TODO: Rename to look_lr_stop
            self.P_sc.stop_wiggle()

        elif command_input == "up":
            self.T_sc.single_servo(13, 1, 7)

        elif command_input == "down":
            self.T_sc.single_servo(13, -1, 7)

        elif command_input == "UDstop":  # TODO: Rename to look_ud_stop
            self.T_sc.stop_wiggle()

        elif command_input == "camera_home":
            self.P_sc.single_servo(12, 0, 7)
            self.T_sc.single_servo(13, 0, 7)

        # Status commands
        elif command_input == "get_info":
            response["title"] = "get_info"
            response["data"] = [info.get_cpu_temp(), info.get_cpu_use(), info.get_ram_info()]

        # Lights
        if 'scan' == command_input:
            pass

        elif 'findColor' == command_input:
            # flask_app.mode_select('findColor')
            self.camera.modeSelect = 'findColor'

        elif 'motionGet' == command_input:
            # flask_app.mode_select('watchDog')
            self.camera.modeSelect = 'watchDog'

        elif 'stopCV' == command_input:
            # flask_app.mode_select('none')
            self.camera.modeSelect = 'none'
        # Single LEDs off (not used for now)
        # switch.switch(1,0)
        # switch.switch(2,0)
        # switch.switch(3,0)

        elif 'KD' == command_input:
            move.command(command_input)

        elif 'automaticOff' == command_input:
            move.command(command_input)

        elif 'automatic' == command_input:
            move.command(command_input)

        elif 'trackLine' == command_input:
            self.camera.mode_select('findlineCV')

        elif 'trackLineOff' == command_input:
            self.camera.mode_select('none')

        elif 'police' == command_input:
            self.RL.police()

        elif 'policeOff' == command_input:
            self.RL.pause()

        # Servo calibration
        if 'SiLeft' in command_input:
            servo_num = int(command_input[7:])
            self.init_pwms[servo_num] = self.init_pwms[servo_num] - 1
            self.legs.set_init_position(servo_num, self.init_pwms[servo_num], True)

        if 'SiRight' in command_input:
            servo_num = int(command_input[7:])
            self.init_pwms[servo_num] = self.init_pwms[servo_num] + 1
            self.legs.set_init_position(servo_num, self.init_pwms[servo_num], True)

        if 'PWMMS' in command_input:
            num_servo = int(command_input[6:])
            config.write("pwm", f"init_pwm{num_servo}", self.init_pwms[num_servo])

        if 'PWMINIT' == command_input:
            for i in range(0, 16):
                self.legs.set_init_position(i, self.init_pwms[i], True)

        if 'PWMD' in command_input:
            reset_pwm = {}
            for i in range(0, 16):
                reset_pwm[f"init_pwm{i}"] = 300
            config.write("pwm", None, reset_pwm)

        """
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

			#CVFL
			elif 'CVFL' == data:
				flask_app.mode_select('findlineCV')

			elif 'CVFLColorSet' in data:
				color = int(data.split()[1])
				flask_app.camera.colorSet(color)

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

		elif isinstance(data, dict):
			if data['title'] == "findColorSet":
				color = data['data']
				flask_app.color_find_set(color[0],color[1],color[2])

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


        return response
