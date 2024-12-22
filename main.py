import sys
import atexit
import signal
import logging
import uvicorn

# Custom modules
from functions import Functions
from camera.opencv import Camera
from web.api import WebApi
from brain.commander import Commander
from servo.base import ServoCtrl
from light.strip import LightStrip
# import switch  # The 3 single LEDs switches, we don't need them for now

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
# logging.getLogger('websockets').setLevel(logging.INFO)

# speed_set = 100
# rad = 0.5
# turnWiggle = 60

# modeSelect = 'none'
# modeSelect = 'PT'

class RobotControl:
	def __init__(self):
		self.legs = None
		self.P_sc = None
		self.T_sc = None
		self.camera = None
		self.light_strip = None
		self.api = None

	def initialize_components(self):
		logger.info("RobotControl.initialize_components")

		try:
			Functions().start()
		except Exception as e:
			logger.error(f"Failed to start Functions: {e}")
			sys.exit(1)
		logger.info("RobotControl.functions started")

		try:
			self.legs = ServoCtrl()
			self.legs.move_init()
		except Exception as e:
			logger.error(f"Failed to initialize ServoCtrl: {e}")
			sys.exit(1)
		logger.info("RobotControl.legs initialized")

		try:
			self.P_sc = ServoCtrl()
			self.P_sc.start()
			self.T_sc = ServoCtrl()
			self.T_sc.start()
		except Exception as e:
			logger.error(f"Failed to initialize Pan/Tilt Servos: {e}")
			sys.exit(1)
		logger.info("RobotControl.camera servo initialized")

		try:
			self.camera = Camera()  # TODO: Check that no one else initializes Camera()!
		except Exception as e:
			logger.error(f"Failed to initialize Camera: {e}")
			sys.exit(1)
		logger.info("RobotControl.camera initialized")

		try:
			self.light_strip = LightStrip()
			self.light_strip.start()
			self.light_strip.stars()
			# RL.breath(70,70,255)
			# RL.rainbow()
		except Exception as e:
			logger.error(f"Failed to initialize LightStrip: {e}")
			sys.exit(1)
		logger.info("RobotControl.light_strip initialized")

		# Commander
		try:
			commander = Commander(
				legs=self.legs,
				P_sc=self.P_sc,
				T_sc=self.T_sc,
				RL=self.light_strip,
				camera=self.camera
			)
		except Exception as e:
			logger.error(f"Failed to initialize Commander: {e}")
			sys.exit(1)
		logger.info("RobotControl.brain initialized")

		# Initialize WebApi
		try:
			self.api = WebApi(self.camera, commander)
		except Exception as e:
			logger.error(f"Failed to initialize WebApi: {e}")
			sys.exit(1)
		logger.info("RobotControl.api initialized")

	def shutdown_components(self):
		logger.info("RobotControl.shutdown_components")
		if self.light_strip:
			try:
				self.light_strip.pause()
			except Exception as e:
				logger.error(f"Error shutting down LightStrip: {e}")

		for c in [self.legs, self.P_sc, self.T_sc]:
			if c:
				try:
					c.shutdown()
				except Exception as e:
					logger.error(f"Error shutting down component {c}: {e}")

		if self.camera:
			try:
				self.camera.stop_thread()
			except Exception as e:
				logger.error(f"Error stopping Camera: {e}")

		logger.info("RobotControl.shutdown_components done")


if __name__ == "__main__":
	logger.info('main.init')

	robot_control = RobotControl()

	def graceful_shutdown(*args):
		logger.info("Executing graceful shutdown...")
		robot_control.shutdown_components()

	atexit.register(graceful_shutdown)
	signal.signal(signal.SIGINT, graceful_shutdown)
	signal.signal(signal.SIGTERM, graceful_shutdown)

	try:
		robot_control.initialize_components()
		logger.info("Starting server with uvicorn")
		uvicorn.run(robot_control.api.app, host="0.0.0.0", port=8000)
	except Exception as e:
		logger.error(f"Unexpected error: {e}")
		graceful_shutdown()
