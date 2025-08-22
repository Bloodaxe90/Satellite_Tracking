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
    config : dict,
    camera_stream: CameraStream,
    fsm: FSM,
    master_dark: np.ndarray,
    results: pd.DataFrame,
    origin_pos: tuple,
    distances: tuple,
    fsm_sleep_time: float,
    amplitude_bounds: tuple,
    kalman_filter: cv2.KalmanFilter,
):

    wait("Start Tracking?")

    learning_iterations_kf = config['kalman_filter']["learning_iterations_kf"]
    insane_threshold = config['kalman_filter']["insane_threshold"]
    feedback_rate = config['general']["feedback_rate"]


    min_amplitude_x, max_amplitude_x, min_amplitude_y, max_amplitude_y = (
        amplitude_bounds
    )
    distance_x, distance_y = distances
    origin_x, origin_y = origin_pos
    largest_contour_search_times = deque(maxlen=10)
    old_dropped_frames = 0
    amplitude_x = 0
    amplitude_y = 0
    start_time = time.time()
    last_time = start_time
    last_sleep_time = start_time
    first_measurement = True
    insane_count = 0

    i = 0
    while True:
        current_time = time.time()
        sample_time = current_time - last_time
        last_time = current_time

        set_transition_matrix(kalman_filter, sample_time)
        kalman_filter.predict()

        raw_frame = camera_stream.read()
        if raw_frame is None:
            time.sleep(0.001)
            continue

        clean_frame = get_clean_frame(raw_frame, master_dark)

        measured_x = None
        measured_y = None

        contours = get_contours(clean_frame)
        avg_largest_contour_search_time = (
            np.mean(largest_contour_search_times, axis=0) if first_measurement else 0
        )
        if contours:
            largest_contour_search_time = time.time()
            largest_contour = get_largest_contour(contours)
            measured_x, measured_y = get_contour_origin(largest_contour)
            largest_contour_search_times.append(
                time.time() - largest_contour_search_time
            )
        else:
            time.sleep(
                avg_largest_contour_search_time
            )  # to keep sample rate consistent for KF

        new_x = measured_x
        new_y = measured_y

        if kalman_filter:
            sane_measurement = (
                is_sane_measurement(kalman_filter, measured_x, measured_y)
                if i > learning_iterations_kf
                else True
            )

            insane_count += -insane_count if sane_measurement else 1

            if insane_count > insane_threshold:
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
                estimated_state = kalman_filter.statePost
                new_x = estimated_state[0, 0]
                new_y = estimated_state[1, 0]

            print(
                f"M {measured_x} {measured_y}\nE {kalman_filter.statePost[0, 0]} {kalman_filter.statePost[1, 0]}\nSane {sane_measurement}"
            )

        delta_x = origin_x - new_x
        delta_y = origin_y - new_y

        results.loc[len(results)] = {
            "measured_X": measured_x,
            "measured_Y": measured_y,
            "estimated_X": kalman_filter.statePost[0, 0] if kalman_filter else new_x,
            "estimated_Y": kalman_filter.statePost[1, 0] if kalman_filter else new_y,
            "delta_x": delta_x,
            "delta_y": delta_y,
            "Time": time.time() - start_time,
        }

        if (i + 1) % feedback_rate == 0 and feedback_rate is not None:
            new_dropped_frames = camera_stream.get_dropped_frames()
            print(
                f"Iteration: {i + 1}, Update Rate: {camera_stream.get_update_rate()}, Sample Rate: {(i + 1) / (time.time() - start_time)}, Dropped Frames: {new_dropped_frames - old_dropped_frames}"
            )
            old_dropped_frames = new_dropped_frames

        current_sleep_time = time.time()
        if current_sleep_time - last_sleep_time >= fsm_sleep_time:
            amplitude_x += delta_x / (distance_x * math.tan(math.radians(50)))
            amplitude_y -= delta_y / (distance_y * math.tan(math.radians(50)))

            amplitude_x = min(max_amplitude_x, max(min_amplitude_x, amplitude_x))
            amplitude_y = min(max_amplitude_y, max(min_amplitude_y, amplitude_y))

            fsm.send_command(
                f"xy={amplitude_x};{amplitude_y}",
                print_sent=False,
                print_received=False,
            )
            last_sleep_time = current_sleep_time

        i += 1
