# import os
import cv2
import numpy as np
# import switch  # The 3 single LEDs switches, we don't need them for now
import datetime
# import PID
import time
import threading
import imutils
import logging
from picamera2 import Picamera2

import servo
from servo.base import ServoCtrl
# from light.strip import LightStrip
from camera.base import BaseCamera
from system.kalman_filter import KalmanFilter
from servo import move

# os.environ["LIBCAMERA_LOG_LEVELS"] = "2"
# logging.getLogger('picamera2').setLevel(logging.INFO)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# light_strip = LightStrip()
# pid = PID.PID()
# pid.SetKp(0.5)
# pid.SetKd(0)
# pid.SetKi(0)

CVRun = 1
linePos_1 = 440
linePos_2 = 380
frameRender = 1
findLineError = 60

colorUpper = np.array([44, 255, 255])
colorLower = np.array([24, 100, 100])

class CVThread(threading.Thread):
	font = cv2.FONT_HERSHEY_SIMPLEX

	kalman_filter_X = KalmanFilter(0.01, 0.1)
	kalman_filter_Y = KalmanFilter(0.01, 0.1)
	P_direction = -1
	T_direction = 1
	P_servo = 12
	T_servo = 13
	P_anglePos = 0
	T_anglePos = 0
	cameraDiagonalW = 64
	cameraDiagonalH = 48
	videoW = 640
	videoH = 480
	Y_lock = 0
	X_lock = 0
	tor = 27

	# TODO: MUST be inited from webServer and passed!
	scGear = ServoCtrl()
	scGear.move_init()
	# Single LED switches, not used now
	# switch.switchSetup()

	def __init__(self, *args, **kwargs):
		self.CVThreading = 0
		self.CVMode = 'none'
		self.imgCV = None

		self.mov_x = None
		self.mov_y = None
		self.mov_w = None
		self.mov_h = None

		self.radius = 0
		self.box_x = None
		self.box_y = None
		self.drawing = 0

		self.findColorDetection = 0

		self.left_Pos1 = None
		self.right_Pos1 = None
		self.center_Pos1 = None

		self.left_Pos2 = None
		self.right_Pos2 = None
		self.center_Pos2 = None

		self.center = None

		super(CVThread, self).__init__(*args, **kwargs)
		self.__flag = threading.Event()
		self.__flag.clear()

		self.avg = None
		self.motionCounter = 0
		self.lastMotionCaptured = datetime.datetime.now()
		self.frameDelta = None
		self.thresh = None
		self.cnts = None

		self.movement = None  # Will be set by Camera class

	def mode(self, invar, img_input):
		self.CVMode = invar
		self.imgCV = img_input
		self.resume()

	def element_draw(self, img_input):
		if self.CVMode == 'none':
			pass

		elif self.CVMode == 'findColor':
			if self.findColorDetection:
				cv2.putText(img_input,'Target Detected',(40,60), CVThread.font, 0.5,(255,255,255),1,cv2.LINE_AA)
				self.drawing = 1
			else:
				cv2.putText(img_input,'Target Detecting',(40,60), CVThread.font, 0.5,(255,255,255),1,cv2.LINE_AA)
				self.drawing = 0

			if self.radius > 10 and self.drawing:
				cv2.rectangle(img_input,(int(self.box_x-self.radius),int(self.box_y+self.radius)),(int(self.box_x+self.radius),int(self.box_y-self.radius)),(255,255,255),1)

		elif self.CVMode == 'watchDog':
			if self.drawing:
				cv2.rectangle(img_input, (self.mov_x, self.mov_y), (self.mov_x + self.mov_w, self.mov_y + self.mov_h), (128, 255, 0), 1)

		return img_input


	def watch_dog(self, img_input):
		logger.info('watchDog.started')

		timestamp = datetime.datetime.now()
		gray = cv2.cvtColor(img_input, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (21, 21), 0)

		if self.avg is None:
			logger.info("starting background model...")
			self.avg = gray.copy().astype("float")
			return 'background model'

		cv2.accumulateWeighted(gray, self.avg, 0.5)
		self.frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(self.avg))

		# threshold the delta image, dilate the thresholded image to fill
		# in holes, then find contours on thresholded image
		self.thresh = cv2.threshold(self.frameDelta, 5, 255,
			cv2.THRESH_BINARY)[1]
		self.thresh = cv2.dilate(self.thresh, None, iterations=2)
		self.cnts = cv2.findContours(self.thresh.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		self.cnts = imutils.grab_contours(self.cnts)
		# logger.info('x')

		logger.info('watchDog: contours')
		logger.info(self.cnts)

		# loop over the contours
		for c in self.cnts:
			# if the contour is too small, ignore it
			if cv2.contourArea(c) < 5000:
				continue
	 
			# compute the bounding box for the contour, draw it on the frame,
			# and update the text
			(self.mov_x, self.mov_y, self.mov_w, self.mov_h) = cv2.boundingRect(c)
			self.drawing = 1
			
			self.motionCounter += 1
			#logger.info(motionCounter)
			#logger.info(text)
			self.lastMotionCaptured = timestamp
			# light_strip.set_color(255,78,0)
			# leds.both_off()
			# leds.red()
			# Single LED switches, not used now
			# switch.switch(1,1)
			# switch.switch(2,1)
			# switch.switch(3,1)

		if (timestamp - self.lastMotionCaptured).seconds >= 0.5:
			logger.info('watchDog: no motion detected')
			# light_strip.set_color(0,78,255)
			# leds.both_off()
			# leds.blue()
			self.drawing = 0
			# Single LED switches, not used now
			# switch.switch(1,0)
			# switch.switch(2,0)
			# switch.switch(3,0)
		self.pause()

	def servo_move(ID, Dir, errorInput):
		if ID == 12:
			errorGenOut = CVThread.kalman_filter_X.kalman(errorInput)
			CVThread.P_anglePos += 0.15*(errorGenOut*Dir)*CVThread.cameraDiagonalW/CVThread.videoW

			if abs(errorInput) > CVThread.tor:
				CVThread.scGear.move_angle(ID, CVThread.P_anglePos)
				CVThread.X_lock = 0
			else:
				CVThread.X_lock = 1
		elif ID == 13:
			error_gen_out = CVThread.kalman_filter_Y.kalman(errorInput)
			CVThread.T_anglePos += 0.15*(error_gen_out*Dir)*CVThread.cameraDiagonalH/CVThread.videoH

			if abs(errorInput) > CVThread.tor:
				CVThread.scGear.move_angle(ID, CVThread.T_anglePos)
				CVThread.Y_lock = 0
			else:
				CVThread.Y_lock = 1
		else:
			logger.info(f"No servoPort {ID} assigned.")


	def find_color(self, frame_image):
		hsv = cv2.cvtColor(frame_image, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, colorLower, colorUpper)#1
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None
		if len(cnts) > 0:
			self.findColorDetection = 1
			c = max(cnts, key=cv2.contourArea)
			((self.box_x, self.box_y), self.radius) = cv2.minEnclosingCircle(c)
			m = cv2.moments(c)
			center = (int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]))
			x = int(self.box_x)
			y = int(self.box_y)
			error_y = 240 - y
			error_x = 320 - x
			CVThread.servo_move(CVThread.P_servo, CVThread.P_direction, -error_x)
			CVThread.servo_move(CVThread.T_servo, CVThread.T_direction, -error_y)

			if CVThread.X_lock == 1 and CVThread.Y_lock == 1:
				# light_strip.set_color(255,78,0)
				# led.both_off()
				# led.red()
				logger.info('CVThread: findColor locked')
			else:
				# light_strip.set_color(0,78,255)
				# led.both_off()
				# led.blue()
				logger.info('CVThread: findColor unlocked')
		else:
			self.findColorDetection = 0
		self.pause()


	def pause(self):
		self.__flag.clear()

	def resume(self):
		self.__flag.set()

	def run(self):
		while 1:
			self.__flag.wait()
			if self.CVMode == 'none':
				servo.move.command('stand')
				servo.move.command('no')
				continue
			elif self.CVMode == 'findColor':
				self.CVThreading = 1
				self.find_color(self.imgCV)
				self.CVThreading = 0
			elif self.CVMode == 'watchDog':
				self.CVThreading = 1
				self.watch_dog(self.imgCV)
				self.CVThreading = 0
			pass


class Camera(BaseCamera):
	video_source = 0
	modeSelect = 'none'
	# modeSelect = 'findColor'
	# modeSelect = 'watchDog'


	def __init__(self, movement_controller=None):
		# if os.environ.get('OPENCV_CAMERA_SOURCE'):
		# 	Camera.set_video_source(int(os.environ['OPENCV_CAMERA_SOURCE']))
		# super(Camera, self).__init__()
		super(Camera, self).__init__()
		# self.picam2 = Picamera2()
		# self.picam2.configure(self.picam2.create_preview_configuration(main={"size": (640, 480)}))
		# self.picam2.start()

		self.movement = movement_controller  # RobotMovement instance
		self.modeSelect = 'none'

	def set_movement_controller(self, movement_controller):
		"""Set the movement controller instance."""
		self.movement = movement_controller
		# Also set it for the CV thread if it exists
		if hasattr(self, 'thread') and self.thread:
			self.thread.movement = movement_controller

	def _handle_movement_command(self, command: str):
		"""Handle movement commands."""
		if self.movement:
			self.movement.command(command)
		else:
			logger.warning("Movement controller not set, ignoring command")

	def color_find_set(self, invarH, invarS, invarV):
		global colorUpper, colorLower
		HUE_1 = invarH+15
		HUE_2 = invarH-15
		if HUE_1>180:HUE_1=180
		if HUE_2<0:HUE_2=0

		SAT_1 = invarS+150
		SAT_2 = invarS-150
		if SAT_1>255:SAT_1=255
		if SAT_2<0:SAT_2=0

		VAL_1 = invarV+150
		VAL_2 = invarV-150
		if VAL_1>255:VAL_1=255
		if VAL_2<0:VAL_2=0

		colorUpper = np.array([HUE_1, SAT_1, VAL_1])
		colorLower = np.array([HUE_2, SAT_2, VAL_2])
		logger.info('Camera: HSV_1:%d %d %d'%(HUE_1, SAT_1, VAL_1))
		logger.info('Camera: HSV_2:%d %d %d'%(HUE_2, SAT_2, VAL_2))
		logger.info(f'Camera: colorUpper {colorUpper}')
		logger.info(f'Camera: colorLower {colorLower}')

	def mode_set(self, invar):
		Camera.modeSelect = invar

	def cv_run_set(self, invar):
		global CVRun
		CVRun = invar

	def line_pos_set_1(self, invar):
		global linePos_1
		linePos_1 = invar

	def line_pos_set_2(self, invar):
		global linePos_2
		linePos_2 = invar

	def rander_set(self, invar):
		global frameRender
		frameRender = invar

	def error_set(self, invar):
		global findLineError
		findLineError = invar

	@staticmethod
	def set_video_source(source):
		Camera.video_source = source

	@staticmethod
	def frames():
		# logger.info(f"Camera source: {Camera.video_source}")
		#
		# camera = cv2.VideoCapture(Camera.video_source)
		# logger.info(f"Camera: {camera}")
		# logger.info(f"Camera.isOpened: {camera.isOpened()}")
		# if not camera.isOpened():
		# 	raise RuntimeError('Could not start camera.')
		#
		# cvt = CVThread()
		# cvt.start()
		camera = Picamera2()
		camera.configure(camera.create_preview_configuration(main={"size": (640, 480)}))
		camera.start()

		while True:
			# # read current frame
			# # _, img = camera.read()
			#
			# for _ in range(10):  # Read 10 frames to allow the camera to stabilize
			# 	_, img = camera.read()
			# 	logger.info(f"Camera: {img}")
			# 	time.sleep(1)
			#
			# if img is None:
			# 	raise RuntimeError('Could not read frame.')
			#
			# if img.all is None:
			# 	continue
			#
			# if Camera.modeSelect == 'none':
			# 	switch.switch(1,0)
			# 	cvt.pause()
			# else:
			# 	if cvt.CVThreading:
			# 		pass
			# 	else:
			# 		cvt.mode(Camera.modeSelect, img)
			# 		cvt.resume()
			# 	try:
			# 		img = cvt.elementDraw(img)
			# 	except:
			# 		pass
			#
			#
			#
			# # encode as a jpeg image and return it
			# if cv2.imencode('.jpg', img)[0]:
			# 	yield cv2.imencode('.jpg', img)[1].tobytes()
			frame = camera.capture_array()
			# Convert the frame to BGR format for OpenCV
			frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

			# Encode the frame in JPEG format
			ret, jpeg = cv2.imencode('.jpg', frame)
			if not ret:
				continue
			yield jpeg.tobytes()


	"""
	def in_center(self, posInput, setCenter):
		# Make the bot move left/right will the center line at posInput
		# is in center, executed like:
		# self.in_center(self.center, 320)
		if posInput and setCenter:
			if posInput > (setCenter + findLineError):
				servo.move.command('right')
				logger.info('CVThread: findLineCtrl right')
				pass
			elif posInput < (setCenter - findLineError):
				servo.move.command('left')
				logger.info('CVThread: findLineCtrl left')
				pass
			else:
				servo.move.command('forward')
				time.sleep(1)
				pass
	"""