import cv2
import numpy as np

from src.kalman_filter.kalman_filter_calibration import set_transition_matrix

def setup_kalman_filter(delta_t: float,
                        model_uncertainty: float = 0.05,
                        measurement_uncertainty: float = 0.5
                        ) -> cv2.KalmanFilter:
    """
    Sets up a Kalman filter to track position and velocity using a constant acceleration model

    Parameters:
        delta_t (float): The time between each transition
        model_uncertainty (float): How much uncertainty we have in the model accurately predicting the next position (default is 0.05) (higher value = less certain)
        measurement_uncertainty (float): How much uncertainty we have in our x, y position measurements representing the actual position of the laser (default is 0.5) (higher value = less certain)

    Returns:
        cv2.KalmanFilter: Configured Kalman filter object
    """

    # States of x position, y position, x velocity, y velocity
    # Measures the x position and y position
    kalman_filter: cv2.KalmanFilter = cv2.KalmanFilter(4, 2)

    kalman_filter.measurementMatrix = np.array([
        [1, 0, 0, 0],  # x position
        [0, 1, 0, 0]   # y position
    ], dtype=np.float32)

    set_transition_matrix(kalman_filter,
                          delta_t)

    # Process noise covariance: Uncertainty in our model
    kalman_filter.processNoiseCov = np.eye(4, dtype=np.float32) * model_uncertainty

    # Measurement noise covariance: Uncertainty in the measurements (from sensor/camera)
    kalman_filter.measurementNoiseCov = np.eye(2, dtype=np.float32) * measurement_uncertainty


    return kalman_filter