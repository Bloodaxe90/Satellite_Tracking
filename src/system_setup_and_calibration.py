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
    config: dict,
    root_dir: str,
    amplitude_bounds: tuple[float, float, float, float]
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
    cam_cfg = config['camera']
    fsm_cfg = config['fsm']
    kf_cfg = config['kalman_filter']

    bins = cam_cfg['bins']
    max_resolution = cam_cfg["max_resolution"]

    camera, master_dark = setup_camera(
        root_dir=root_dir,
        sdk_lib_name=cam_cfg['sdk_lib_name'],
        camera_id=cam_cfg['camera_id'],
        bins=bins,
        gain=cam_cfg['gain'],
        exposure=cam_cfg['exposure'],
        resolution=max_resolution,
        start_pos=tuple(cam_cfg['start_pos']),
        dark_frames=cam_cfg['dark_frames'],
    )

    fsm: FSM = setup_fsm(fsm_cfg["port"], fsm_cfg["baudrate"])

    if fsm_cfg["generate_origin"]:
        wait("Ready to generate origin?")
        origin_frames = [
            get_clean_frame(camera.capture(), master_dark) for _ in range(10)
        ]
        origin_image = np.median(origin_frames, axis=0).astype(np.uint8)
        contours = get_contours(origin_image)
        largest_contour = get_largest_contour(contours)
        origin_pos = get_contour_origin(largest_contour)
    else:
        origin_pos = (
        (max_resolution[0] / bins) / 2, (max_resolution[1] / bins) / 2)

    use_fsm_rio = fsm_cfg["use_fsm_roi"]

    extended_message = ", FSM RIO and Amplitude Bounds" if use_fsm_rio else ""
    wait(
        f"Ready to calibrate distance from camera{extended_message}.\n"
        "Please ensure the laser points to around the center of the camera and can be\n"
        "seen in frame (check resources/images/test_image.jpg)\n"
    )

    distances = get_distance_to_camera(
        camera=camera,
        master_dark=master_dark,
        origin_pos=origin_pos,
        fsm=fsm,
        amplitude_bounds=amplitude_bounds,
    )

    if use_fsm_rio:
        calibrated_start_pos, calibrated_resolution, amplitude_bounds = (
            get_fsm_roi_and_amplitude_bounds(
                camera=camera,
                master_dark=master_dark,
                fsm=fsm,
                max_resolution=max_resolution,
            )
        )

        set_roi(camera, calibrated_resolution, calibrated_start_pos, bins)

        master_dark = get_master_dark(camera, fsm_cfg["dark_frames"], "fsm_master_dark.npy")

    camera_stream = CameraStream(camera).start()

    fsm_sleep_time = get_response_time(
        camera_stream, master_dark, fsm, amplitude_bounds
    )

    initial_sample_time = 1 / camera_stream.get_update_rate()
    kalman_filter = setup_kalman_filter(
        delta_t=initial_sample_time,
        model_uncertainty=kf_cfg["model_uncertainty"],
        measurement_uncertainty=kf_cfg["measurement_uncertainty"],
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
