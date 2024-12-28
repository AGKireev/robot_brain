import sys
import atexit
import signal
import logging
import uvicorn
import asyncio
from mpu6050 import mpu6050

# Custom modules
from camera.opencv import Camera
from web.api import WebApi
from brain.commander import Commander
from servo.base import ServoCtrl
from light.strip import LightStrip
from servo import move  # Changed from 'movement import move' to 'servo import move'
# import switch  # The 3 single LEDs switches, we don't need them for now

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
# logging.getLogger('websockets').setLevel(logging.INFO)

# speed_set = 100
# rad = 0.5
# turnWiggle = 60

# modeSelect = 'none'
# modeSelect = 'PT'

class Robot:
	def __init__(self):
		self.servo_legs = None
		self.servo_camera_lr = None
		self.servo_camera_ud = None
		self.camera = None
		self.light_strip = None
		self.api = None
		self.sensor = None  # MPU6050 sensor, accelerometer and gyroscope
		self.movement = None  # Robot movement controller

	def init_components(self):
		logger.info("Robot.init_components")

		try:
			self.sensor = mpu6050(0x68)
		except Exception as e:
			logger.error(f"Robot.Failed to initialize sensor: {e}")
			sys.exit(1)
		logger.info("Robot.sensor initialized")

		try:
			self.servo_legs = ServoCtrl()
			self.servo_legs.move_init()
		except Exception as e:
			logger.error(f"Robot.Failed to initialize Legs Servos: {e}")
			sys.exit(1)
		logger.info("Robot.legs servos initialized")

		try:
			self.servo_camera_lr = ServoCtrl()
			self.servo_camera_lr.start()
			self.servo_camera_ud = ServoCtrl()
			self.servo_camera_ud.start()
		except Exception as e:
			logger.error(f"Robot.Failed to initialize Camera Servos: {e}")
			sys.exit(1)
		logger.info("Robot.camera servo initialized")

		try:
			# Initialize movement controller first
			self.movement = move.RobotMovement(self.servo_legs)
			self.movement.set_init_positions(self.servo_legs.init_positions)
		except Exception as e:
			logger.error(f"Robot.Failed to initialize Movement Controller: {e}")
			sys.exit(1)
		logger.info("Robot.movement controller initialized")

		try:
			# Initialize camera with movement controller
			self.camera = Camera(self.movement)
		except Exception as e:
			logger.error(f"Robot.Failed to initialize Camera: {e}")
			sys.exit(1)
		logger.info("Robot.camera initialized")

		try:
			self.light_strip = LightStrip()
			self.light_strip.start()
			self.light_strip.stars()
		except Exception as e:
			logger.error(f"Robot.Failed to initialize LightStrip: {e}")
			sys.exit(1)
		logger.info("Robot.light_strip initialized")

		# Commander
		try:
			commander = Commander(
				servo_legs=self.servo_legs,
				servo_camera_lr=self.servo_camera_lr,
				servo_camera_ud=self.servo_camera_ud,
				light_strip=self.light_strip,
				camera=self.camera
			)
		except Exception as e:
			logger.error(f"Robot.Failed to initialize Commander: {e}")
			sys.exit(1)
		logger.info("Robot.brain initialized")

		# Initialize WebApi
		try:
			self.api = WebApi(self.camera, commander)
		except Exception as e:
			logger.error(f"Robot.Failed to initialize WebApi: {e}")
			sys.exit(1)
		logger.info("Robot.api initialized")

	def shutdown_components(self):
		logger.info("Robot.shutdown_components")
		# if self.movement:
		# 	try:
		# 		self.movement.cleanup()
		# 	except Exception as e:
		# 		logger.error(f"Robot.Error cleaning up movement controller: {e}")

		if self.light_strip:
			try:
				self.light_strip.pause()
			except Exception as e:
				logger.error(f"Robot.Error shutting down LightStrip: {e}")

		for c in [self.servo_legs, self.servo_camera_lr, self.servo_camera_ud]:
			if c:
				try:
					c.shutdown()
				except Exception as e:
					logger.error(f"Robot.Error shutting down component {c}: {e}")

		if self.camera:
			try:
				self.camera.stop_thread()
			except Exception as e:
				logger.error(f"Robot.Error stopping Camera: {e}")

		logger.info("Robot.shutdown_components done")


if __name__ == "__main__":
	logger.info('main.init')

	robot = Robot()

	def cancel_all_tasks():
		tasks = asyncio.all_tasks()
		for t in tasks:
			logger.info(f"main.Running task: {t}")
			t.cancel()
		logger.info("main.All asyncio tasks have been canceled.")

	def graceful_shutdown(*args):
		logger.info("main.Executing graceful shutdown...")
		try:
			robot.shutdown_components()
			cancel_all_tasks()
		finally:
			logger.info("main.Exiting process.")
			sys.exit(0)

	atexit.register(graceful_shutdown)
	signal.signal(signal.SIGINT, graceful_shutdown)
	signal.signal(signal.SIGTERM, graceful_shutdown)

	try:
		robot.init_components()
		logger.info("main.Start server")
		uvicorn.run(robot.api.app, host="0.0.0.0", port=8000)
	except Exception as e:
		logger.error(f"main.Unexpected error: {e}")
		graceful_shutdown()
