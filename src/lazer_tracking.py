import time
from collections import deque

import cv2
import numpy as np
import zwoasi as asi

from src.camera.camera_stream import CameraStream
from src.camera.image_processing import get_clean_frame, get_contours, \
    get_largest_contour, get_contour_origin, get_redness_frame
from src.fsm.fsm import FSM


def lazer_tracking(camera: asi.Camera,
                   master_dark: np.ndarray,
                   camera_stream: CameraStream,
                   origin_pos: tuple[float, float],
                   amplitude_per_pixel: tuple,
                   fsm: FSM,
                   kalman_filter: cv2.KalmanFilter,
                   buffer_capacity: int = 1,
                   kernel_size: int = 3,
                   colour: bool = False):
    """
    Tracks a laser point by adjusting a Fast Steering Mirror (FSM)
    in real time to keep the point centered

    Parameters:
        camera (asi.Camera): Camera object used to capture images
        master_dark (np.ndarray): Dark frame for subtracting sensor noise
        camera_stream (CameraStream): Threaded stream wrapper for continuous frame access
        origin_pos (tuple[float, float]): Target (x, y) pixel coordinates of the desired laser origin
        amplitude_per_pixel (tuple): Conversion ratio from pixels to FSM amplitude for (x, y)
        fsm (FSM): FSM controller used to send mirror control signals
        kalman_filter (cv2.KalmanFilter): Kalman filter instance for prediction and correction
        buffer_capacity (int): Number of frames to median average for smoothing (default is 1)
        kernel_size (int): Kernel size used in frame cleaning for morphological opening(default is 3)
        colour (bool): Whether to use redness channel (True) or grayscale (False)
    """

    fsm.send_command("control strategy feedforward")
    # Buffer to store past frames for smoothing
    frame_buffer: deque = deque(maxlen=buffer_capacity)

    origin_x, origin_y = origin_pos
    amplitude_per_pixel_x, amplitude_per_pixel_y = amplitude_per_pixel

    amplitude_x: float = 0
    amplitude_y: float = 0

    last_time = time.time()

    while True:
        current_time = time.time()
        sample_time = current_time - last_time
        last_time = current_time

        # Update the transition matrix with actual change in time
        kalman_filter.transitionMatrix[0, 2] = sample_time
        kalman_filter.transitionMatrix[1, 3] = sample_time

        # Predict the next state based on the model
        kalman_filter.predict()

        raw_frame = camera_stream.read()

        if raw_frame is None:
            # If no frame is available wait and retry
            time.sleep(0.001)
            continue

        clean_frame = get_clean_frame(raw_frame, master_dark, kernel_size)

        # Convert to redness channel if color tracking is enabled
        if colour and camera.get_camera_property()["IsColorCam"]:
            clean_frame = get_redness_frame(clean_frame)

        # If using frame buffer update it and compute median
        if buffer_capacity > 1:
            frame_buffer.append(clean_frame)
            clean_frame = (np.median(np.stack(frame_buffer, axis=0), axis=0)
                           .astype(np.uint8))

        contours = get_contours(clean_frame)

        if contours:
            # Identify the largest contour (assumed to be the laser)
            largest_contour = get_largest_contour(contours)

            measured_x, measured_y = get_contour_origin(largest_contour)

            # If this is the first measurement, initialize Kalman state
            if kalman_filter.statePost is None:
                kalman_filter.errorCovPost = np.eye(4, dtype=np.float32) * 1
                kalman_filter.statePost = np.array(
                    [[measured_x], [measured_y], [0], [0]], dtype=np.float32)

            else:
                # Correct Kalman state using new measurement
                kalman_filter.correct(
                    np.array([[measured_x], [measured_y]], dtype=np.float32))

            # Get the post correction estimate
            estimated_state = kalman_filter.statePost
            new_x = estimated_state[0, 0]
            new_y = estimated_state[1, 0]

        else:
            # Get the pre correction prediction
            predicted_state = kalman_filter.statePre
            new_x = predicted_state[0, 0]
            new_y = predicted_state[1, 0]

        # Calculate pixel shift from origin
        delta_x = origin_x - new_x
        delta_y = origin_y - new_y

        # Convert pixel displacement to FSM amplitude
        amplitude_x += delta_x * amplitude_per_pixel_x
        amplitude_y += delta_y * amplitude_per_pixel_y

        # Ensure amplitude stays within physical limits
        amplitude_x = max(-1.0, min(1.0, amplitude_x))
        amplitude_y = max(-1.0, min(1.0, amplitude_y))

        # Send control signals to FSM to adjust beam position
        fsm.send_command(f"xy={amplitude_x};{amplitude_y}",
                         print_received=False)