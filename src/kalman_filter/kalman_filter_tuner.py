import math
import numpy as np
import pandas as pd
import time

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
from src.kalman_filter.kalman_filter_setup import setup_kalman_filter
from src.utils.general import wait


def tuner(
    config: dict,
    camera_stream: CameraStream,
    fsm: FSM,
    results: pd.DataFrame,
    master_dark: np.ndarray,
    origin_pos: tuple,
    distances: tuple,
    fsm_sleep_time: float,
    amplitude_bounds: tuple,
):

    sim_fsm_cfg = config['simulation_fsm']
    tun_cfg = config['tuner_params']

    print("Tuning Parameters")

    amp_incr_x, amp_incr_y = sim_fsm_cfg["amplitude_step_size"]
    min_amplitude_x, max_amplitude_x, min_amplitude_y, max_amplitude_y = (
        amplitude_bounds
    )
    distance_x, distance_y = distances
    origin_x, origin_y = origin_pos
    model_uncertainties = tun_cfg["initial_model_uncertainties"]
    measurement_uncertainties = tun_cfg["initial_measurement_uncertainties"]

    wait("Start?")

    tune_iterations = tun_cfg["search_iterations"]
    for i in range(tune_iterations):

        print(f"{i + 1}. MOU {model_uncertainties}, MEU {measurement_uncertainties}")

        for model_uncertainty in model_uncertainties:
            for measurement_uncertainty in measurement_uncertainties:

                print(
                    f"Tested parameters: MOU {model_uncertainty}, MEU {measurement_uncertainty}"
                )
                error_x = []
                error_y = []
                sane_measurements = []

                fsm.send_command(f"xy=0;0", False, False)
                time.sleep(1)

                initial_sample_time = 1 / camera_stream.get_update_rate()
                kalman_filter = setup_kalman_filter(
                    delta_t=initial_sample_time,
                    model_uncertainty=model_uncertainty,
                    measurement_uncertainty=measurement_uncertainty,
                )
                start_time = time.time()
                last_time = start_time
                first_measurement = True
                last_sleep_time = start_time
                amplitude_x = 0
                amplitude_y = 0
                for i in range(tun_cfg["tune_iterations"]):

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

                    contours = get_contours(clean_frame)
                    largest_contour = get_largest_contour(contours)
                    measured_x, measured_y = get_contour_origin(largest_contour)

                    sane_measurement = is_sane_measurement(
                        kalman_filter, measured_x, measured_y
                    )

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
                    estimated_state = kalman_filter.statePost
                    estimated_x = estimated_state[0, 0]
                    estimated_y = estimated_state[1, 0]

                    delta_x = origin_x - measured_x
                    delta_y = origin_y - measured_y

                    error_x.append(abs(measured_x - estimated_x))
                    error_y.append(abs(measured_y - estimated_y))
                    sane_measurements.append(sane_measurement)
                    if fsm:
                        current_sleep_time = time.time()
                        if current_sleep_time - last_sleep_time >= fsm_sleep_time:
                            amplitude_x += delta_x / (
                                distance_x * math.tan(math.radians(50))
                            )
                            amplitude_y -= delta_y / (
                                distance_y * math.tan(math.radians(50))
                            )

                            assert (
                                min_amplitude_x < amplitude_x < max_amplitude_x
                                and min_amplitude_y < amplitude_y < max_amplitude_y
                            ), f"Amplitude must be within the FSM's limits {amplitude_bounds}"
                            fsm.send_command(
                                f"xy={amplitude_x};{amplitude_y}",
                                print_sent=False,
                                print_received=False,
                            )
                            last_sleep_time = current_sleep_time

                    if sim_fsm_cfg["enable"]:
                        amplitude_x += amp_incr_x
                        amplitude_y += amp_incr_y
                        fsm.send_command(
                            f"xy={amplitude_x};{amplitude_y}", False, False
                        )

                results.loc[len(results)] = {
                    "model_uncertainty": model_uncertainty,
                    "measurement_uncertainty": measurement_uncertainty,
                    "error_x": np.mean(error_x, axis=0),
                    "error_y": np.mean(error_y, axis=0),
                    "sane_rate": np.sum(sane_measurements) / tune_iterations,
                }

        rmse = np.sqrt(results["error_x"] ** 2 + results["error_y"] ** 2)
        normalized_rmse = (rmse - rmse.min()) / (rmse.max() - rmse.min())
        cost = (tun_cfg["error_weight"] * normalized_rmse) + (
            tun_cfg["sane_weight"] * (1 - results["sane_rate"])
        )
        best_index = np.argmin(cost, axis=0)

        center_value = results["model_uncertainty"][best_index]
        half_range = center_value * tun_cfg["model_reduction_factor"]
        new_space = np.linspace(
            center_value - half_range,
            center_value + half_range,
            len(model_uncertainties),
        )
        model_uncertainties = np.array([max(x, 1e-9) for x in new_space]).astype(
            np.float32
        )

        center_value = results["measurement_uncertainty"][best_index]
        half_range = center_value * tun_cfg["measurement_reduction_factor"]
        new_space = np.linspace(
            center_value - half_range,
            center_value + half_range,
            len(measurement_uncertainties),
        )

        measurement_uncertainties = np.array([max(x, 1e-9) for x in new_space]).astype(
            np.float32
        )
