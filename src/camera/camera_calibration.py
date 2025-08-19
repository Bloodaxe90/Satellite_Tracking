import os
import time
from typing import Tuple, Any

import cv2
import numpy as np
import zwoasi as asi

from src.camera.camera_stream import CameraStream
from src.camera.image_processing import get_contours, get_largest_contour, \
    get_contour_origin, get_clean_frame, get_redness_frame
from src.fsm.fsm import FSM


def set_colour(camera: asi.Camera, colour: bool):
    """
    Sets the camera's image type based on whether it supports colour and whether colour is requested

    Parameters:
        camera (asi.Camera): The camera object to configure
        colour (bool): True if colour output is desired, False for grayscale

    If the camera supports colour and colour is requested, the image type is set to RGB
    If the camera does not support colour but colour is requested, a warning is printed
    In all other cases, the image type is set to RAW8 (grayscale)
    """
    if camera.get_camera_property()["IsColorCam"] and colour:
        camera.set_image_type(asi.ASI_IMG_RGB24)
    else:
        if colour:
            print("Camera does not have colour capabilities")
        camera.set_image_type(asi.ASI_IMG_RAW8)


def reset_settings(camera: asi.Camera):
    """
    Resets all writable camera controls to their default values

    Parameters:
        camera (asi.Camera): The camera object to reset

    If a control supports auto mode, it is first disabled before resetting the value
    """
    controls = camera.get_controls()

    for control_key in sorted(controls.keys()):
        control = controls[control_key]

        # Only reset controls that can be changed
        if control['IsWritable']:
            # If the control supports auto mode, disable it before resetting
            if control['IsAutoSupported']:
                camera.set_control_value(control['ControlType'], 0, auto=False)

            camera.set_control_value(control['ControlType'],
                                     control['DefaultValue'])

    print("Settings Reset to Default\n")


def get_optimum_gain_exposure(camera: asi.Camera,
                              sleep_interval: int = 0.1,
                              minimum_matches: int = 10
                              ) -> tuple[int, int]:
    """
    Automatically finds and returns stable gain and exposure settings for the camera

    Parameters:
        camera (asi.Camera): The camera to configure
        sleep_interval (int): Time (in seconds) to wait between checks. Default is 0.1
        minimum_matches (int): Number of times gain and exposure must remain the same
                               before being considered stable. Default is 5

    Returns:
        tuple[int, int]: A tuple containing the final stable gain and exposure values
    """

    # Enable auto mode for gain and exposure so the camera can adjust them
    camera.set_control_value(
        control_type=asi.ASI_GAIN,
        value=0,
        auto=True
    )
    camera.set_control_value(
        control_type=asi.ASI_EXPOSURE,
        value=0,
        auto=True
    )

    old_gain = None
    old_exposure = None
    matches = 0

    # Start capturing video frames so auto calibration can happen
    camera.start_video_capture()

    while True:
        time.sleep(sleep_interval)
        camera.capture_video_frame()

        new_gain, _ = camera.get_control_value(asi.ASI_GAIN)
        new_exposure, _ = camera.get_control_value(asi.ASI_EXPOSURE)

        # Check if the values have stopped changing
        if old_gain == new_gain and old_exposure == new_exposure:
            matches += 1
        else:
            print(f"Current Gain: {new_gain}, Exposure: {new_exposure}")
            matches = 0

        # If values have stayed the same for enough checks, stop
        if matches >= minimum_matches:
            print("Gain and Exposure have stabilized")
            break

        old_gain = new_gain
        old_exposure = new_exposure

    camera.stop_video_capture()

    return new_gain, new_exposure


def set_roi(camera: asi.Camera,
            resolution: tuple,
            start_pos: tuple,
            bins: int ):
    """
    Sets the Region of Interest (ROI) for the camera

    Parameters:
        camera (asi.Camera): The camera object to configure
        resolution (tuple): The desired resolution (width, height) BEFORE binning
        start_pos (tuple): The (x, y) starting position of the ROI
        bins (int): Binning factor (must be >= 1)

    The function sets the ROI and prints out the actual settings after applying them
    """

    assert bins >= 1, "Bin size must be a positive integer"

    # Set the ROI using the given start position and resolution and adjust for binning
    camera.set_roi(start_x=start_pos[0],
                   start_y=start_pos[1],
                   width=resolution[0],
                   height=resolution[1],
                   bins=bins)

    start_x, start_y, width, height = camera.get_roi()

    print(f"ROI Set:\n"
          f"    Start X: {start_x}\n"
          f"    Start Y: {start_y}\n"
          f"    Resolution: {width}x{height}\n"
          f"    Bins: {bins}\n")



def get_master_dark(camera: asi.Camera, num_frames: int, file_name: str = "master_dark.npy") -> np.ndarray:
    """
    Returns the master dark frame either by loading an existing file or creating a new one

    Parameters:
        camera (asi.Camera): The camera object to capture dark frames
        num_frames (int): Number of dark frames to capture when creating a new one

    Returns:
        np.ndarray: The master dark frame

    Note:
        This function prompts the user to ensure the lens cap is on during dark frame capture
    """
    # Path to save/load the master dark frame
    file_path = os.path.join(
        os.path.dirname(os.getcwd()),
        'resources',
        'dark_frame',
        file_name
    )

    # Make sure we are capturing at least one frame
    assert num_frames > 0, f"Must have at least 1 dark frame, not {num_frames}"

    len_cap: str = input(
        "\nCreate new dark frame?\n"
        "A new dark frame is necessary if camera settings have changed\n\n"
        "Y/n? "
    ).lower()

    if len_cap == "y" or len_cap == "yes":
        print("Creating new master dark")
        dark_frames = [camera.capture() for _ in range(num_frames)]

        assert dark_frames, "No dark frames were captured, ensure camera is properly connected"

        # Take the median of all frames to reduce noise and get a clean dark frame
        master_dark = np.median(dark_frames, axis=0).astype(np.uint8)

        np.save(
            file_path,
            master_dark
        )
        print("New Master Dark created and saved\n")
    else:
        master_dark = np.load(file_path)
        print("Master Dark loaded\n")

    return master_dark

def print_controls(camera: asi.Camera):
    """
    Prints the current control setting of the camera

    Parameters:
        camera (asi.Camera): The camera to print controls of
    """
    controls = camera.get_controls()
    for control_key in sorted(controls.keys()):
        print(f"    {control_key}")
        for property_key in sorted(controls[control_key].keys()):
            print(f"      - {property_key} {repr(controls[control_key][property_key])}")

def set_gain_exposure(camera: asi.Camera,
                      gain: int,
                      exposure: int):
    """
    Sets the gain and exposure settings for the camera

    Parameters:
        camera (asi.Camera): The camera to configure
        gain (int): The desired gain value (ignored if auto mode is chosen)
        exposure (int): The desired exposure value in microseconds (ignored if auto mode is chosen)

    The user is prompted to choose between using automatic calibration and their own manual values
    """

    auto_calibrate: str = input(
        "Auto Calibrate Gain and Exposure?\n"
        "This will let the camera pick the Gain and Exposure values for you,\n"
        "overriding any values provided.\n\n"
        "Y/n?\n"
    ).lower()

    if auto_calibrate == "y" or auto_calibrate == "yes":
        print("Using optimum gain & exposure settings")
        # Lets the camera decide the best values automatically
        gain, exposure = get_optimum_gain_exposure(camera)
    else:
        print("Using provided gain & exposure settings")

    print(f"Gain: {gain}, Exposure: {exposure}µs\n")

    # Set the gain and exposure to the automatic/manual values provided and turns of auto mode
    camera.set_control_value(asi.ASI_GAIN, gain, auto=False)
    camera.set_control_value(asi.ASI_EXPOSURE, exposure, auto=False)
