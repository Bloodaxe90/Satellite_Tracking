import cv2
import numpy as np


def is_sane_measurement(kalman_filter: cv2.KalmanFilter,
                        measured_x: float,
                        measured_y: float,
                        threshold: float = 9.21 # Chi-squared value for a p-value of 0.01.
                        ):

    measurement = np.array([[measured_x], [measured_y]],
                           dtype=np.float32)
    predicted_measurement = kalman_filter.measurementMatrix @ kalman_filter.statePre
    difference = measurement - predicted_measurement

    # Uncertainty in the models prediction projected from a 6x6 to a 2x2 matrix
    prediction_uncertainty = ((kalman_filter.measurementMatrix @ kalman_filter.errorCovPre) @
                              kalman_filter.measurementMatrix.T)
    # Total uncertainty in the difference between the measured and predicted positions
    difference_uncertainty = np.linalg.inv(prediction_uncertainty + kalman_filter.measurementNoiseCov)

    # Squared mahalanobis distance taking into account the uncertainty in the difference
    mahalanobis_distance_squared = ((difference.T @ difference_uncertainty) @ difference)[0, 0]

    # Checks weather the measurement falls within 99% confidence of the models prediction
    return mahalanobis_distance_squared < threshold


def set_transition_matrix(kalman_filter: cv2.KalmanFilter,
                          delta_t: float,):

    # Setting the model for constant velocity
    kalman_filter.transitionMatrix = np.array(
        [[1, 0, delta_t, 0], # new_x = (1 * old_x) + (old_vx * ∆t)
                [0, 1, 0, delta_t], # new_y = (1 * old_y) + (old_vy * ∆t)
                [0, 0, 1, 0], # new_vx = old_vx * 1
                [0, 0, 0, 1]], # new_vy = old_vy * 1
        dtype=np.float32)




# For constant acceleration
#
# def set_transition_matrix(kalman_filter: cv2.KalmanFilter,
#                           delta_t: float,):
#
#     # Setting the model for constant acceleration
#     half_delta_t_squared = 0.5 * (delta_t ** 2)
#     kalman_filter.transitionMatrix = np.array([
#         [1, 0, delta_t, 0, half_delta_t_squared, 0], # new_x = (1 * old_x) + (old_vx * ∆t) + (0.5 * old_ax * ∆t^2)
#         [0, 1, 0, delta_t, 0, half_delta_t_squared], # new_y = (1 * old_y) + (old_vy * ∆t) + (0.5 * old_ay * ∆t^2)
#         [0, 0, 1, 0, delta_t, 0], # new_vx = (1 * old_vx) + (old_vx * ∆t)
#         [0, 0, 0, 1, 0, delta_t], # new_vy = (1 * old_vy) + (old_ax * ∆t)
#         [0, 0, 0, 0, 1, 0], # new_ax = 1 * old_ax
#         [0, 0, 0, 0, 0, 1] # new_vy = 1 * old_ay
#     ], dtype=np.float32)
#
# def set_process_noise_covariance(kalman_filter: cv2.KalmanFilter,
#                                  delta_t: float,
#                                  model_uncertainty: float):
#
#     # Process noise covariance: Uncertainty in our model
#     # more complex than a constant velocity model as the uncertainty in models
#     # acceleration, velocity and position changes at different rates
#     half_delta_t_squared = 0.5 * (delta_t ** 2)
#     tmp = np.array([
#         [half_delta_t_squared],
#         [half_delta_t_squared],
#         [delta_t],
#         [delta_t],
#         [1.0],
#         [1.0]
#     ], dtype=np.float32)
#     kalman_filter.processNoiseCov = (tmp @ tmp.T) * model_uncertainty