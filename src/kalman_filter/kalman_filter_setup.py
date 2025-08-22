import cv2
import numpy as np

from src.kalman_filter.kalman_filter_calibration import set_transition_matrix


def setup_kalman_filter(
    delta_t: float,
    model_uncertainty: float = 0.05,
    measurement_uncertainty: float = 0.5,
) -> cv2.KalmanFilter:
    """
    Sets up a Kalman filter to track position and velocity using a constant velocity model

    Parameters:
        delta_t (float): The time between each transition
        model_uncertainty (float): How much uncertainty we have in the model accurately predicting the next position (default is 0.05) (higher value = less certain)
        measurement_uncertainty (float): How much uncertainty we have in our x, y position measurements representing the actual position of the laser (default is 0.5) (higher value = less certain)

    Returns:
        cv2.KalmanFilter: Configured Kalman filter object
    """

    print("Setting up Kalman Filter\n")
    # States of x position, y position, x velocity, y velocity
    # Measures the x position and y position
    kalman_filter: cv2.KalmanFilter = cv2.KalmanFilter(8, 2)

    # 1. State Transition Matrix (A) - The Physics Model
    # This is the core of the Constant Jerk model.
    # p_new = p + v*dt + 0.5*a*dt^2 + (1/6)*j*dt^3
    # v_new = v + a*dt + 0.5*j*dt^2
    # a_new = a + j*dt
    # j_new = j  (The "Constant Jerk" assumption)
    dt2 = 0.5 * delta_t**2
    dt3 = (1/6) * delta_t**3
    kalman_filter.transitionMatrix = np.array([
        [1, 0, delta_t, 0,   dt2,   0,   dt3,   0],  # x
        [0, 1, 0, delta_t,   0,   dt2,   0,   dt3],  # y
        [0, 0, 1,       0, delta_t,   0,   dt2,   0],  # vx
        [0, 0, 0,       1,   0, delta_t,   0,   dt2],  # vy
        [0, 0, 0,       0,   1,       0, delta_t,   0],  # ax
        [0, 0, 0,       0,   0,       1,   0, delta_t],  # ay
        [0, 0, 0,       0,   0,       0,   1,       0],  # jx
        [0, 0, 0,       0,   0,       0,   0,       1]   # jy
    ], dtype=np.float32)

    # Measurement Matrix (H) = Maps state to measurement
    kalman_filter.measurementMatrix = np.array([
        [1, 0, 0, 0, 0, 0, 0, 0],  # x position
        [0, 1, 0, 0, 0, 0, 0, 0]   # y position
    ], dtype=np.float32)

    # Process Noise Covariance (Q) = Uncertainty in our model
    kalman_filter.processNoiseCov = np.eye(8, dtype=np.float32) * model_uncertainty

    jerk_process_noise = 1e-6

    kalman_filter.processNoiseCov[6, 6] = jerk_process_noise
    kalman_filter.processNoiseCov[7, 7] = jerk_process_noise

    # Measurement Noise Covariance (R) = Uncertainty in our measurements
    kalman_filter.measurementNoiseCov = np.eye(2, dtype=np.float32) * measurement_uncertainty

    return kalman_filter
