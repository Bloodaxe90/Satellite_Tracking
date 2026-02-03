from collections import deque

import cv2
import numpy as np
import pandas as pd


def is_sane_measurement(
    kalman_filter: cv2.KalmanFilter,
    measured_x: float,
    measured_y: float,
    threshold: float = 500,  # Chi-squared value for p=0.01 with 2 DOF
) -> bool | np.ndarray[tuple[int, ...], np.dtype[bool]]:
    """
    Validate whether a measurement is statistically consistent with the Kalman filter prediction.

    Args:
        kalman_filter (cv2.KalmanFilter): The Kalman filter
        measured_x (float): Measured x-coordinate
        measured_y (float): Measured y-coordinate
        threshold (float, optional): Chi-squared cutoff value for rejecting unlikely
                                     measurements. Default = 9.21 (99% confidence).

    Returns:
        bool:
            - True if the measurement is within the confidence threshold.
            - False if it is likely an outlier or invalid.
    """

    if measured_x is None or measured_y is None:
        return False

    measurement = np.array([[measured_x], [measured_y]], dtype=np.float32)
    predicted_measurement = kalman_filter.measurementMatrix @ kalman_filter.statePre
    difference = measurement - predicted_measurement

    # Uncertainty in the models prediction projected from a 6x6 to a 2x2 matrix
    prediction_uncertainty = (
        kalman_filter.measurementMatrix @ kalman_filter.errorCovPre
    ) @ kalman_filter.measurementMatrix.T
    # Total uncertainty in the difference between the measured and predicted positions
    difference_uncertainty = np.linalg.inv(
        prediction_uncertainty + kalman_filter.measurementNoiseCov
    )

    # Squared mahalanobis distance taking into account the uncertainty in the difference
    mahalanobis_distance_squared = (
        (difference.T @ difference_uncertainty) @ difference
    )[0, 0]

    # Checks weather the measurement falls within 99% confidence of the models prediction
    return mahalanobis_distance_squared < threshold

def is_sane_measurement2(
        old_measured_x,
        old_measured_y,
        measured_x,
        measured_y,
        std_devs: int,
        distances: deque):
    if not distances:
        return True

    if measured_x is None or measured_y is None:
        return False

    current_distance_x = abs(old_measured_x - measured_x)
    current_distance_y = abs(old_measured_y - measured_y)
    mean_distance_x = np.mean([i[0] for i in distances], axis = 0)
    mean_distance_y = np.mean([i[1] for i in distances], axis = 0)
    std_dev_distance_x = np.std([i[0] for i in distances], axis = 0)
    std_dev_distance_y = np.std([i[1] for i in distances], axis = 0)

    threshold_x = mean_distance_x + (std_dev_distance_x * std_devs)
    threshold_y = mean_distance_y + (std_dev_distance_y * std_devs)

    return current_distance_x <= threshold_x or current_distance_y <= threshold_y

def set_transition_matrix(
    kalman_filter: cv2.KalmanFilter,
    delta_t: float,
) -> None:
    """
    Set the Kalman filter's state transition matrix for a constant velocity model.

    Args:
        kalman_filter (cv2.KalmanFilter): The Kalman filter instance to update
        delta_t (float): Time step (in seconds) between samples
    """
    dt = delta_t
    dt2 = 0.5 * dt ** 2  # 1/2 * dt^2
    dt3 = (1 / 6) * dt ** 3  # 1/6 * dt^3, which is 1/3! * dt^3

    # The 8x8 State Transition Matrix for the Constant Jerk model
    kalman_filter.transitionMatrix = np.array([
        [1, 0, dt, 0, dt2, 0, dt3, 0],  # Position x
        [0, 1, 0, dt, 0, dt2, 0, dt3],  # Position y
        [0, 0, 1, 0, dt, 0, dt2, 0],  # Velocity x
        [0, 0, 0, 1, 0, dt, 0, dt2],  # Velocity y
        [0, 0, 0, 0, 1, 0, dt, 0],  # Acceleration x
        [0, 0, 0, 0, 0, 1, 0, dt],  # Acceleration y
        [0, 0, 0, 0, 0, 0, 1, 0],  # Jerk x
        [0, 0, 0, 0, 0, 0, 0, 1]  # Jerk y
    ], dtype=np.float32)

