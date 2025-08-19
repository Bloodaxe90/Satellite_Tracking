import os

import numpy as np
import zwoasi as asi

from src.camera.camera_calibration import (
    reset_settings,
    set_roi,
    set_colour,
    set_gain_exposure, get_master_dark, print_controls
)


def setup_camera(root_dir: str,
                 sdk_lib_name: str,
                 camera_id: int,
                 resolution: tuple,
                 start_pos: tuple,
                 bins: int,
                 gain: int,
                 exposure: int,
                 num_frames: int,
                 colour: bool = False) -> tuple[asi.Camera, np.ndarray]:
    """
    Initialize and configure a ZWO ASI camera

    Args:
        root_dir (str): Root directory of the project
        sdk_lib_name (str): Filename of the ZWO SDK library
        camera_id (int): Index of the camera to initialize
        resolution (tuple): Target resolution (width, height)
        start_pos (tuple): Starting (x, y) position for ROI
        bins (int): Binning factor
        gain (int): Manual gain setting (ignored if auto-calibration is enabled)
        exposure (int): Manual exposure setting in microseconds (ignored if auto-calibration is enabled)
        num_frames (int): Number of dark frames to capture when creating a new one
        colour (bool): Whether to set the camera to color mode if it is supported (Defaults to False)

    Returns:
        tuple[asi.Camera, np.ndarray]: Configured ASI camera object and master dark frame
    """

    # Resolve absolute path to SDK
    sdk_path = os.path.normpath(os.path.join(root_dir, 'resources', 'sdk_lib', sdk_lib_name))

    try:
        asi.init(sdk_path)
    except IOError:
        raise RuntimeError(f"Cannot locate SDK at path: {sdk_path}")

    num_cameras = asi.get_num_cameras()
    if camera_id >= num_cameras:
        raise ValueError(f"Camera with ID {camera_id} not found. Available IDs: {list(range(num_cameras))}")

    camera = asi.Camera(camera_id)

    # Reset all camera controls to defaults
    reset_settings(camera)

    # Set Region of Interest (ROI)
    set_roi(camera, resolution, start_pos, bins)

    # Set max USB bandwidth
    max_bandwidth = camera.get_controls()['BandWidth']['MaxValue']
    camera.set_control_value(asi.ASI_BANDWIDTHOVERLOAD, max_bandwidth, auto= False)

    # Disable in-camera dark subtraction to work with raw data (this is done manually using my own code)
    camera.disable_dark_subtract()

    # Manually or auto set gain & exposure based on users choice
    set_gain_exposure(camera, gain, exposure)

    # Set color mode if supported and requested
    set_colour(camera, colour)

    # Enable High Speed Mode
    camera.set_control_value(asi.ASI_HIGH_SPEED_MODE, 1)

    print_controls(camera)

    # Capture test image to verify configuration
    image_dir = os.path.normpath(os.path.join(root_dir, 'resources', 'images'))
    os.makedirs(image_dir, exist_ok=True)  # Ensure the directory exists

    test_image_path = os.path.join(image_dir, "test_image.jpg")
    print("Taking Test Image:")
    test_image = camera.capture(filename=f"{image_dir}/test_image.jpg")
    print(f"Saved test image to {test_image_path}\n")

    master_dark = get_master_dark(camera, num_frames)

    assert master_dark.shape == test_image.shape, "Master dark frame doesn't match current settings, please restart the program and create a new one"

    return camera, master_dark
