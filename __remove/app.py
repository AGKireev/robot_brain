import os
from flask import Flask, Response
from flask_cors import CORS
import threading
import logging
from camera.opencv import Camera

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class WebApp:
    def __init__(self):
        logger.info('WebApp: __init__')
        self.app = Flask(__name__)
        
        CORS(self.app, supports_credentials=True)

        logger.info('WebApp: Camera')
        self.camera = Camera()

        logger.info('WebApp: Routes')
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.setup_routes()
        
        logger.info('WebApp: __init__ done')

    def setup_routes(self):
        @self.app.route('/video_feed')
        def video_feed():
            return Response(self.gen(self.camera),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

    @staticmethod
    def gen(camera):
        """
        Generator function that yields video frames
        """
        while True:
            frame = camera.get_frame()
            # Yield a byte string that represents a single frame in MJPEG format
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    @staticmethod
    def mode_select(mode_input):
        Camera.modeSelect = mode_input

    def color_find_set(self, h, s, v):
        """
        Method to set the HSV color for object detection
        """
        self.camera.color_find_set(h, s, v)

    def thread(self):
        """
        Run the Flask app as a separate thread
        """
        self.app.run(host='0.0.0.0', threaded=True)

    def start_thread(self):
        logger.info('WebApp: start_thread')

        fps_threading = threading.Thread(target=self.thread)
        fps_threading.daemon = False  # in order to prevent abrupt termination of the thread
        fps_threading.start()
        logger.info('WebApp: start_thread done')
