import time
import threading
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class CameraEvent(object):
    """An Event-like class that signals all active clients when a new frame is
    available.
    """

    def __init__(self):
        # Dictionary of client events; keys are thread identifiers
        self.events = {}

    def wait(self, timeout=None):
        """Invoked from each client's thread to wait for the next frame."""
        ident = threading.get_ident()
        if ident not in self.events:
            # this is a new client
            # add an entry for it in the self.events dict
            # each entry has two elements, a threading.Event() and a timestamp
            self.events[ident] = [threading.Event(), time.time()]
        return self.events[ident][0].wait(timeout)

    def set(self):
        """Invoked by the camera thread when a new frame is available."""
        now = time.time()
        remove = None
        for ident, event in self.events.items():
            if not event[0].isSet():
                # if this client's event is not set, then set it
                # also update the last set timestamp to now
                event[0].set()
                event[1] = now
            else:
                # if the client's event is already set, it means the client
                # did not process a previous frame
                # if the event stays set for more than 5 seconds, then assume
                # the client is gone and remove it
                if now - event[1] > 5:
                    remove = ident
        if remove:
            del self.events[remove]

    def clear(self):
        """Invoked from each client's thread after a frame was processed."""
        self.events[threading.get_ident()][0].clear()


class BaseCamera(object):
    """An abstract base class for camera implementations."""

    thread = None  # background thread that reads frames from camera
    frame = None  # current frame is stored here by background thread
    last_access = 0  # time of last client access to the camera
    event = CameraEvent()

    def __init__(self, timeout=10):
        """Start the background camera thread if it isn't running yet."""
        if BaseCamera.thread is None:
            logger.info("BaseCamera: init")
            BaseCamera.last_access = time.time()

            # start background frame thread
            try:
                BaseCamera.thread = threading.Thread(target=self._thread)
                BaseCamera.thread.start()

                # wait until frames are available
                start_time = time.time()
                while self.get_frame(timeout=timeout) is None:
                    if time.time() - start_time > timeout:
                        logger.error("BaseCamera: could not start camera")
                        raise RuntimeError("BaseCamera: could not start camera")
                    time.sleep(0)
            except Exception as e:
                logger.error(f"BaseCamera: Error initializing camera: {e}")
                BaseCamera.thread = None

    def get_frame(self, timeout=None):
        """Return the current camera frame."""
        BaseCamera.last_access = time.time()

        # wait for a signal from the camera thread
        if not BaseCamera.event.wait(timeout=timeout):
            return None
        BaseCamera.event.clear()

        return BaseCamera.frame

    @staticmethod
    def frames():
        """"Generator that returns frames from the camera."""
        raise NotImplementedError('BaseCamera: Must be implemented by subclasses.')

    @classmethod
    def _thread(cls):
        """Camera background thread."""
        logger.info('BaseCamera: Starting camera thread.')
        try:
            frames_iterator = cls.frames()
            for frame in frames_iterator:
                BaseCamera.frame = frame
                BaseCamera.event.set()  # send signal to clients
                time.sleep(0)

                # if there hasn't been any clients asking for frames in
                # the last 10 seconds then stop the thread
                if time.time() - BaseCamera.last_access > 10:
                    frames_iterator.close()
                    logger.warning('BaseCamera: Stopping camera thread due to inactivity.')
                    break
        except Exception as e:
            logger.exception(f"Error in camera thread: {e}")
        finally:
            BaseCamera.thread = None

    def stop_thread(self):
        """Stop the background camera thread."""
        logger.info("BaseCamera: Stopping thread.")
        self.last_access = 0  # Reset last access to ensure the thread stops.
        self.event.set()  # Wake up the thread if it's waiting for a frame.
