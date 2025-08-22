import cv2
import numpy as np


def is_sane_measurement(
    kalman_filter: cv2.KalmanFilter,
    measured_x: float,
    measured_y: float,
    threshold: float = 9.21,  # Chi-squared value for p=0.01 with 2 DOF
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
    kalman_filter.transitionMatrix = np.array(
        [
            [1, 0, delta_t, 0],  # new_x = (1 * old_x) + (old_vx * ∆t)
            [0, 1, 0, delta_t],  # new_y = (1 * old_y) + (old_vy * ∆t)
            [0, 0, 1, 0],  # new_vx = old_vx * 1
            [0, 0, 0, 1],
        ],  # new_vy = old_vy * 1
        dtype=np.float32,
    )
