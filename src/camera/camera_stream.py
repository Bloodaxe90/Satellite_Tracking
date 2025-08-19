import threading
import time
from collections import deque

import numpy as np
import zwoasi as asi


class CameraStream:
    """
    A class that continuously captures frames from an ASI camera using a background thread
    """

    def __init__(self, camera: asi.Camera):
        """
        Initializes the CameraStream object

        Parameters:
            camera (asi.Camera): The ASI camera object to capture frames from
        """
        super().__init__()
        self.camera = camera
        self.latest_frame = None  # Store the latest frame
        self.lock = threading.Lock()
        self.capture_time_buffer = deque(maxlen=6)  # Stores frame capture times to later compute FPS
        self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
        self.stopped = False

    def start(self):
        """
        Starts the background thread and enables video mode
        """
        print("Starting camera stream thread\n")
        self.camera.start_video_capture()
        self.stopped = False
        self.update_thread.start()
        time.sleep(1)
        return self

    def update_loop(self):
        """
        The method that runs in the background thread continuously
        capturing frames and saving them to latest_frame
        """
        while not self.stopped:
            frame = self.camera.capture_video_frame()
            with self.lock:
                self.latest_frame = frame
                self.capture_time_buffer.append(time.time())  # Record current time for later FPS calculation

        # When stopping, end the camera video capture
        self.camera.stop_video_capture()
        print("Camera stream thread stopped")

    def get_update_rate(self) -> float:
        """
        Computes and returns the approximate rate at which the latest frame is updated from the timestamps buffer.

        Returns:
            float: Estimated update rate second.
        """
        with self.lock:
            if len(self.capture_time_buffer) < 2:
                print("Buffer is not full enough to compute update rate")
                return 0.0

            time_elapsed: float = self.capture_time_buffer[1] - self.capture_time_buffer[0]

            if time_elapsed != 0:
                return (len(self.capture_time_buffer) - 1) / time_elapsed
            else:
                print("Cannot divide by 0. (time elapsed is 0)")
                return 0.0

    def read(self) -> np.ndarray:
        """
        Returns the most recent frame captured by the camera

        Returns:
            np.ndarray: The latest video frame (or None if not yet available)
        """
        with self.lock:
            return self.latest_frame

    def stop(self):
        """
        Stops the camera stream and waits for the background thread to finish
        """
        print("Stopping camera stream thread")
        self.stopped = True
        self.update_thread.join()
