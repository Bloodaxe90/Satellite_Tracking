import time

import cv2
import numpy as np
import zwoasi

from src.camera.camera_calibration import set_roi, get_master_dark
from src.camera.camera_setup import setup_camera
from src.camera.camera_stream import CameraStream
from src.camera.image_processing import (
    get_clean_frame,
    get_contours,
    get_largest_contour,
    get_contour_origin,
)
from src.fsm.fsm import FSM
from src.fsm.fsm_calibration import (
    get_distance_to_camera,
    get_response_time,
    get_fsm_roi_and_amplitude_bounds,
)
from src.fsm.fsm_setup import setup_fsm
from src.kalman_filter.kalman_filter_setup import setup_kalman_filter
from src.utils.general import wait


def system_setup_and_calibration(
    config: dict, root_dir: str, amplitude_bounds: tuple[float, float, float, float]
) -> tuple[
    zwoasi.Camera,
    CameraStream,
    np.ndarray,
    tuple[float, float],
    FSM,
    float,
    tuple[float, float],
    tuple[float, float, float, float],
    cv2.KalmanFilter,
]:
    """
    Performs full system setup and calibration

    Program Flow:
    1. Set up the Camera and calculate master dark
    2. Set up the FSM
    3. Calculate the origin position:
        If `generate_origin` is enabled compute the origin as the lasers current position
        Else use the center of the image as the origin
    4. Calibrate distance between the FSM and the Camera
    5. If FSM ROI is enabled:
        - Calibrate ROI and amplitude bounds
        - Update camera ROI and master dark frame
        - Adjust the origin
    6. Start the camera stream
    7. Calibrate Cameras response time to changes in the FSM
    8. If Kalman filter is enabled set it up Else set it to None

    Args:
        config (dict): Initial configuration of input parameters
        root_dir (str): Root directory of the project
        amplitude_bounds (tuple[float, float, float, float]): Initial FSM amplitude limits (x_min, x_max, y_min, y_max).

    Returns:
        tuple:
            - zwoasi.Camera: Initialized camera object
            - CameraStream: Initialized and started camera stream object for reading frames
            - np.ndarray: Master dark frame for noise subtraction.
            - tuple[float, float]: Center position of the laser spot in the camera frame
            - FSM: Initialized FSM object.
            - float: Average FSM Camera response time in seconds
            - tuple[float, float]: Distances from FSM to camera
            - tuple[float, float, float, float]: Calibrated amplitude bounds for FSM
            - cv2.KalmanFilter: Configured Kalman filter instance, or None if disabled
    """

    cam_cfg = config["camera"]
    fsm_cfg = config["fsm"]
    kf_cfg = config["kalman_filter"]

    # Camera setup
    bins = cam_cfg["bins"]
    max_resolution = cam_cfg["max_resolution"]
    initial_resolution = tuple([int(val / bins) for val in max_resolution])

    camera, master_dark = setup_camera(
        root_dir=root_dir,
        sdk_lib_name=cam_cfg["sdk_lib_name"],
        camera_id=cam_cfg["camera_id"],
        bins=bins,
        gain=cam_cfg["gain"],
        exposure=cam_cfg["exposure"],
        resolution=initial_resolution,
        start_pos=tuple(cam_cfg["start_pos"]),
        dark_frames=cam_cfg["dark_frames"],
    )

    # FSM setup
    fsm: FSM = setup_fsm(fsm_cfg["port"], fsm_cfg["baudrate"])

    # Origin detection
    if fsm_cfg["generate_origin"]:
        wait("Ready to generate origin?")
        # Capture multiple frames and average to reduce noise
        origin_frames = [
            get_clean_frame(camera.capture(), master_dark) for _ in range(10)
        ]
        origin_image = np.median(origin_frames, axis=0).astype(np.uint8)

        contours = get_contours(origin_image)
        assert contours, "No Contours were found"

        largest_contour = get_largest_contour(contours)
        origin_pos = get_contour_origin(largest_contour)

        print(f"Origin: X {origin_pos[0]}, Y {origin_pos[1]}")
    else:
        # Default origin is the frames center
        origin_pos = ((max_resolution[0] / bins) / 2, (max_resolution[1] / bins) / 2)

    # Distance calibration
    use_fsm_rio = fsm_cfg["use_fsm_roi"]

    extended_message = ", FSM RIO and Amplitude Bounds" if use_fsm_rio else ""
    wait(
        f"Ready to calibrate distance from camera{extended_message}.\n"
        "Please ensure the laser points around the center of the camera and is visible\n"
        "(check resources/images/test_image.jpg)\n"
    )

    distances = get_distance_to_camera(
        camera=camera,
        master_dark=master_dark,
        origin_pos=origin_pos,
        fsm=fsm,
        amplitude_bounds=amplitude_bounds,
    )

    # FSM ROI calibration and amplitude bound correction
    if use_fsm_rio:
        calibrated_start_pos, calibrated_resolution, amplitude_bounds = (
            get_fsm_roi_and_amplitude_bounds(
                camera=camera,
                master_dark=master_dark,
                fsm=fsm,
                max_resolution=max_resolution,
            )
        )

        # Set new ROI on camera
        set_roi(camera, calibrated_resolution, calibrated_start_pos, bins)

        # Adjust origin to new ROI coordinates
        origin_pos = (
            origin_pos[0] - calibrated_start_pos[0],
            origin_pos[1] - calibrated_start_pos[1],
        )

        print(f"Adjusted Origin: X {origin_pos[0]}, Y {origin_pos[1]}\n")

        time.sleep(0.1)

        # Re generate master dark for new ROI
        master_dark = get_master_dark(
            camera, cam_cfg["dark_frames"], "fsm_master_dark.npy"
        )

        assert (
            master_dark.shape == camera.capture().shape
        ), "Master dark frame doesn't match current settings, restart program and regenerate"

    # Camera stream
    camera_stream = CameraStream(camera).start()

    # FSM Camera response time calibration
    fsm_sleep_time = get_response_time(
        camera_stream, master_dark, fsm, amplitude_bounds
    )

    # Kalman filter setup
    initial_sample_time = 1 / camera_stream.get_update_rate()
    kalman_filter = (
        setup_kalman_filter(
            delta_t=initial_sample_time,
            model_uncertainty=kf_cfg["model_uncertainty"],
            measurement_uncertainty=kf_cfg["measurement_uncertainty"],
        )
        if kf_cfg["enabled"]
        else None
    )

    return (
        camera,
        camera_stream,
        master_dark,
        origin_pos,
        fsm,
        fsm_sleep_time,
        distances,
        amplitude_bounds,
        kalman_filter,
    )
