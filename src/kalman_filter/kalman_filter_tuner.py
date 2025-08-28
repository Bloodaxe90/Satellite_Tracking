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
) -> tuple[float, float]:
    """
    Tunes the Kalman filter parameters for optimal tracking performance using an iterative search

    This function tests different combinations of model uncertainty (MOU) and
    measurement uncertainty (MEU) parameters in the Kalman filter. It evaluates
    each parameter set by measuring:
        - The error between measured and estimated positions (RMSE)
        - The fraction of sane measurements (measurements that fall
          within the expected uncertainty bounds)

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

    Returns:
        tuple[float, float]: Best model uncertainty value, Best measurement uncertainty value
    """
    lin_fsm_cfg = config["linear_fsm"]
    tun_cfg = config["tuner_params"]
    error_weight = tun_cfg["error_weight"]
    sane_weight = tun_cfg["sane_weight"]
    linear_fsm = lin_fsm_cfg["enabled"]

    print("Tuning Parameters")

    amplitude_step_x, amplitude_step_y = lin_fsm_cfg["amplitude_step_size"]
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
                error_x, error_y, sane_measurements = [], [], []

                # Reset FSM
                fsm.send_command("xy=0;0", False, False)
                time.sleep(1)

                # Initialize Kalman filter
                initial_sample_time = 1 / camera_stream.get_update_rate()
                kalman_filter = setup_kalman_filter(
                    delta_t=initial_sample_time,
                    model_uncertainty=model_uncertainty,
                    measurement_uncertainty=measurement_uncertainty,
                )
                last_time = time.time()
                first_measurement = True
                last_sleep_time = last_time
                amplitude_x = amplitude_y = 0

                for _ in range(tune_iterations):
                    # Update Kalman filter timing
                    current_time = time.time()
                    sample_time = current_time - last_time
                    last_time = current_time
                    set_transition_matrix(kalman_filter, sample_time)
                    kalman_filter.predict()

                    # Get frame
                    raw_frame = camera_stream.read()
                    if raw_frame is None:
                        time.sleep(0.001)
                        continue
                    clean_frame = get_clean_frame(raw_frame, master_dark)

                    # Extract dot position
                    contours = get_contours(clean_frame)
                    assert contours, "No contours were found"
                    largest_contour = get_largest_contour(contours)
                    measured_x, measured_y = get_contour_origin(largest_contour)

                    # Check measurement sanity
                    sane_measurement = is_sane_measurement(
                        kalman_filter, measured_x, measured_y
                    )

                    # Initialize filter on first measurement or correct filter
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

                    # Estimated position
                    estimated_state = kalman_filter.statePost
                    estimated_x, estimated_y = (
                        estimated_state[0, 0],
                        estimated_state[1, 0],
                    )

                    # Tracking Error
                    error_x.append(abs(measured_x - estimated_x))
                    error_y.append(abs(measured_y - estimated_y))
                    sane_measurements.append(sane_measurement)

                    # FSM correction
                    if time.time() - last_sleep_time >= fsm_sleep_time:
                        delta_x, delta_y = origin_x - measured_x, origin_y - measured_y
                        amplitude_x += delta_x / (
                            distance_x * math.tan(math.radians(50))
                        )
                        amplitude_y -= delta_y / (
                            distance_y * math.tan(math.radians(50))
                        )

                        # Prevent exceeding FSM amplitude limits
                        assert (
                            min_amplitude_x < amplitude_x < max_amplitude_x
                            and min_amplitude_y < amplitude_y < max_amplitude_y
                        ), f"Amplitude must be within FSM limits {amplitude_bounds}"

                        fsm.send_command(
                            f"xy={amplitude_x};{amplitude_y}",
                            print_sent=False,
                            print_received=False,
                        )
                        last_sleep_time = time.time()

                    # Optional linear FSM stepping to simulate satellite
                    if linear_fsm:
                        amplitude_x += amplitude_step_x
                        amplitude_y += amplitude_step_y
                        fsm.send_command(
                            f"xy={amplitude_x};{amplitude_y}",
                            print_sent=False,
                            print_received=False,
                        )

                # Record results
                results.loc[len(results)] = {
                    "model_uncertainty": model_uncertainty,
                    "measurement_uncertainty": measurement_uncertainty,
                    "error_x": np.mean(error_x),
                    "error_y": np.mean(error_y),
                    "sane_rate": np.sum(sane_measurements) / tune_iterations,
                }

        # Re center search space around best parameters
        rmse = np.sqrt(results["error_x"] ** 2 + results["error_y"] ** 2)
        normalized_rmse = (rmse - rmse.min()) / (rmse.max() - rmse.min())
        cost = (error_weight * normalized_rmse) + (
            sane_weight * (1 - results["sane_rate"])
        )
        best_index = np.argmin(cost)

        # Narrow model uncertainty range
        center_value = results["model_uncertainty"][best_index]
        half_range = center_value * tun_cfg["model_reduction_factor"]
        model_uncertainties = np.array(
            [
                max(x, 1e-9)
                for x in np.linspace(
                    center_value - half_range,
                    center_value + half_range,
                    len(model_uncertainties),
                )
            ]
        ).astype(np.float32)

        # Narrow measurement uncertainty range
        center_value = results["measurement_uncertainty"][best_index]
        half_range = center_value * tun_cfg["measurement_reduction_factor"]
        measurement_uncertainties = np.array(
            [
                max(x, 1e-9)
                for x in np.linspace(
                    center_value - half_range,
                    center_value + half_range,
                    len(measurement_uncertainties),
                )
            ]
        ).astype(np.float32)

    # Final best parameters
    rmse = np.sqrt(results["error_x"] ** 2 + results["error_y"] ** 2)
    normalized_rmse = (rmse - rmse.min()) / (rmse.max() - rmse.min())
    results["cost"] = (error_weight * normalized_rmse) + (
        sane_weight * (1 - results["sane_rate"])
    )

    best_model_uncertainty = np.array(results["model_uncertainty"])[
        results["cost"].idxmin()
    ]
    best_measurement_uncertainty = np.array(results["measurement_uncertainty"])[
        results["cost"].idxmin()
    ]

    print(
        f"Best Parameters:\n"
        f"    Model Uncertainty: {best_model_uncertainty}\n"
        f"    Measurement Uncertainty: {best_measurement_uncertainty}\n"
    )

    return best_model_uncertainty, best_measurement_uncertainty
