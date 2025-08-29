import math
from collections import deque

import cv2
import numpy as np
import time

import pandas as pd

from src.camera.camera_stream import CameraStream
from src.camera.image_processing import (
    get_clean_frame,
    get_contours,
    get_contour_origin,
    get_largest_contour,
)
from src.fsm.fsm import FSM
from src.kalman_filter.kalman_filter_calibration import (
    is_sane_measurement,
    set_transition_matrix,
)
from src.utils.general import wait


def lazer_tracking(
    config: dict,
    camera_stream: CameraStream,
    fsm: FSM,
    master_dark: np.ndarray,
    results: pd.DataFrame,
    origin_pos: tuple[float, float],
    distances: tuple[float, float],
    fsm_sleep_time: float,
    amplitude_bounds: tuple[float, float, float, float],
    kalman_filter: cv2.KalmanFilter,
):
    """
    Track a laser spot in the camera feed and control the FSM to keep it centered

    Program Flow:
    1. Acquire clean frames from the camera stream
    2. Find center position of largest contour in the frame
    3. If Kalman filter is enabled:
        Check sanity of measurement and
        If insane threshold reached use KFs estimations or
        Else use the actual measurements
    4. Compute positional error relative to the origin
    5. Send corrective commands to the FSM at a calibrated rate
    6. If linear fsm is enabled send commands to move the FSM linearly
    7. Record measurements, estimates, and errors into a results DataFrame

    Args:
        config (dict): Initial configuration of input parameters
        camera_stream (CameraStream): Camera stream for reading frames
        fsm (FSM): FSM used to send movement commands
        results (pd.DataFrame): DataFrame to record tuning outcomes
        master_dark (np.ndarray): Master dark frame for noise subtraction
        origin_pos (tuple): Center position of the laser dot
        distances (tuple): calibrated distances from FSM to Camera in for both axis
        fsm_sleep_time (float): Minimum time (seconds) between FSM updates
        amplitude_bounds (tuple): amplitude limits to prevent overdriving the FSM or moving the laser outside the FSMs ROI
        kalman_filter (cv2.KalmanFilter): Kalman filter instance, or None if disabled

    Returns:
        None – runs indefinitely until externally stopped.
    """

    lin_fsm_cfg = config["linear_fsm"]
    kf_cfg = config["kalman_filter"]
    general_cfg = config["general"]

    linear_fsm = lin_fsm_cfg["enabled"]
    learning_iterations_kf = kf_cfg["learning_iterations"]
    insane_threshold = kf_cfg["insane_threshold"]
    feedback_rate = (
        None if general_cfg["feedback_rate"] == "None" else general_cfg["feedback_rate"]
    )

    if linear_fsm:
        print("Linear FSM is ON")

    wait("Start Tracking?")

    # Extract bounds & calibration values
    min_amplitude_x, max_amplitude_x, min_amplitude_y, max_amplitude_y = (
        amplitude_bounds
    )
    distance_x, distance_y = distances
    origin_x, origin_y = origin_pos

    # State tracking variables
    largest_contour_search_times = deque(maxlen=10)
    old_dropped_frames = 0
    amplitude_x = 0
    amplitude_y = 0
    amplitude_step_x, amplitude_step_y = lin_fsm_cfg["amplitude_step_size"]

    # Timing
    start_time = time.time()
    last_time = start_time
    last_sleep_time = start_time

    # Flags
    first_measurement = True
    insane_count = 0

    # Main tracking loop
    i = 0
    while True:
        # Timing for sample rate
        current_time = time.time()
        sample_time = current_time - last_time
        last_time = current_time

        # Kalman prediction step
        if kalman_filter:
            set_transition_matrix(kalman_filter, sample_time)
            kalman_filter.predict()

        # Acquire frame
        raw_frame = camera_stream.read()
        if raw_frame is None:
            time.sleep(0.001)  # Prevents busy wait
            continue
        clean_frame = get_clean_frame(raw_frame, master_dark)

        measured_x, measured_y = None, None

        # Contour detection
        contours = get_contours(clean_frame)
        avg_largest_contour_search_time = (
            np.mean(largest_contour_search_times, axis=0)
            if not first_measurement
            else 0
        )

        if contours:
            t0 = time.time()
            largest_contour = get_largest_contour(contours)
            measured_x, measured_y = get_contour_origin(largest_contour)
            largest_contour_search_times.append(time.time() - t0)
        else:
            # Keep KF sampling time consistent
            time.sleep(avg_largest_contour_search_time)

        new_x, new_y = measured_x, measured_y

        # Kalman correction
        if kalman_filter:
            sane_measurement = (
                is_sane_measurement(kalman_filter, measured_x, measured_y)
                if i > learning_iterations_kf
                else True
            )

            insane_count += -insane_count if sane_measurement else 1

            if (
                insane_count < insane_threshold
                or measured_x is not None
                or measured_y is not None
            ):
                if first_measurement:
                    first_measurement = False
                    kalman_filter.errorCovPost = np.eye(4, dtype=np.float32)
                    kalman_filter.statePost = np.array(
                        [[measured_x], [measured_y], [0], [0]], dtype=np.float32
                    )
                else:
                    kalman_filter.correct(
                        np.array([[measured_x], [measured_y]], dtype=np.float32)
                    )
            else:
                # Fall back on KF prediction if measurement is insane
                estimated_state = kalman_filter.statePost
                new_x, new_y = estimated_state[0, 0], estimated_state[1, 0]

        assert new_x is not None and new_y is not None, "No Contours were found"

        # Compute error relative to origin
        delta_x = origin_x - new_x
        delta_y = origin_y - new_y

        # Log results
        results.loc[len(results)] = {
            "measured_X": measured_x,
            "measured_Y": measured_y,
            "estimated_X": kalman_filter.statePost[0, 0] if kalman_filter else new_x,
            "estimated_Y": kalman_filter.statePost[1, 0] if kalman_filter else new_y,
            "delta_x": delta_x,
            "delta_y": delta_y,
            "Time": time.time() - start_time,
        }

        # Feedback on dropped frames & update rate
        if feedback_rate is not None and (i + 1) % feedback_rate == 0:
            new_dropped_frames = camera_stream.get_dropped_frames()
            print(
                f"Iteration: {i + 1}, "
                f"Update Rate: {camera_stream.get_update_rate()}, "
                f"Sample Rate: {(i + 1) / (time.time() - start_time)}, "
                f"Dropped Frames: {new_dropped_frames - old_dropped_frames}"
            )
            old_dropped_frames = new_dropped_frames

        # FSM correction
        if time.time() - last_sleep_time >= fsm_sleep_time:
            amplitude_x += delta_x / (distance_x * math.tan(math.radians(50)))
            amplitude_y -= delta_y / (distance_y * math.tan(math.radians(50)))

            # Clip within safe amplitude bounds
            amplitude_x = min(max_amplitude_x, max(min_amplitude_x, amplitude_x))
            amplitude_y = min(max_amplitude_y, max(min_amplitude_y, amplitude_y))

            fsm.send_command(
                f"xy={amplitude_x};{amplitude_y}",
                print_sent=False,
                print_received=False,
            )
            last_sleep_time = time.time()

        i += 1

        # Optional linear FSM stepping to simulate satellite
        if linear_fsm:
            amplitude_x += amplitude_step_x
            amplitude_y += amplitude_step_y
            fsm.send_command(
                f"xy={amplitude_x};{amplitude_y}",
                print_sent=False,
                print_received=False,
            )
