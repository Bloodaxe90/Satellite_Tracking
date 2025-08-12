import math
from typing import Tuple

import cv2
import numpy as np
import zwoasi as asi

from src.camera.image_processing import get_redness_frame, get_clean_frame, \
    get_contours, get_contour_origin, get_largest_contour
from src.fsm.fsm import FSM

def get_amplitude_per_pixel(camera: asi.Camera,
                            master_dark: np.ndarray,
                            origin_pos: tuple[float, float],
                            fsm: FSM,
                            amplitude: float = 0.04,
                            num_frames: int = 10,
                            colour: bool = False) -> tuple:
    """
    Measures how many pixels the laser dot moves per FSM amplitude step
    for both x and y axes

    Parameters:
        camera (asi.Camera): The camera capturing the laser
        master_dark (np.ndarray): A dark frame to remove sensor noise
        origin_pos (tuple): The original (x, y) position of the laser dot
        fsm (FSM): The FSM device controlling laser direction
        amplitude (float): The amplitude of the signal to send to FSM
        num_frames (int): Number of frames to average for calibration
        colour (bool): If True, uses redness method to isolate the laser dot (Defaults to False)

    Returns:
        tuple[float, float]: The amplitude per pixel for the x and y directions
    """

    amplitudes_per_pixel = []  # List to store amplitude per pixel for x and y
    moved_pos = []             # List to store measured positions after movement
    fsm.send_command(f"xy=0;0", print_received= False)

    assert -1.0 < amplitude < 1.0 and amplitude != 0, \
        "Amplitude must be within the FSM's limits (-1.0, 1.0) and not zero"

    for i, axis in enumerate(["x", "y"]):

        # Move FSM to desired calibration position
        fsm.send_command(f"{axis}={amplitude}", print_received= False)

        # Capture and clean frames to find laser dot position
        calibrate_frames = [
            get_clean_frame(camera.capture(), master_dark) for _ in range(num_frames)
        ]

        # If camera is in colour mode and we are using colour, isolate the red channel
        if camera.get_camera_property()["IsColorCam"] and colour:
            calibrate_frames = [
                get_redness_frame(calibrated_frame) for calibrated_frame in calibrate_frames
            ]

        # Reset FSM position after frames captured
        fsm.send_command(f"xy=0;0", print_received= False)

        # Average all captured frames to reduce noise in the result
        calibrate_image = np.median(calibrate_frames, axis=0).astype(np.uint8)

        # Find contours and calculate the new position
        contours = get_contours(calibrate_image)
        largest_contour = get_largest_contour(contours)
        calibrated_pos = get_contour_origin(largest_contour)

        moved_pos.append(calibrated_pos[i])  # Store new position for axis

        # Calculate how many pixels the dot moved
        delta_pos = abs(origin_pos[i] - calibrated_pos[i])

        assert delta_pos != 0, (
            f"Error: FSM did not move for the {axis} axis. Check connection or signal."
        )

        amplitudes_per_pixel.append(amplitude / delta_pos)

    print(f"Origin Pos X: {origin_pos[0]} Y: {origin_pos[1]}")
    print(f"Moved Pos X: {moved_pos[0]} Y: {moved_pos[1]}")
    return tuple(amplitudes_per_pixel)

def get_alignment_offset_angle(camera: asi.Camera,
                                          master_dark: np.ndarray,
                                          origin_pos: tuple[float, float],
                                          fsm: FSM,
                                          amplitude: float = 0.04,
                                          num_frames: int = 10,
                                          colour: bool = False) -> tuple[
    float, ...]:

    fsm.send_command(f"xy=0;0", print_received= False)
    origin_x, origin_y = origin_pos
    assert -1.0 < amplitude < 1.0 and amplitude != 0, \
        "Amplitude must be within the FSM's limits (-1.0, 1.0) and not zero"
    angles = []
    for axis in ["x", "y"]:

        # Move FSM to desired calibration position
        fsm.send_command(f"{axis}={amplitude}", print_received= False)

        # Capture and clean frames to find laser dot position
        calibrate_frames = [
            get_clean_frame(camera.capture(), master_dark) for _ in range(num_frames)
        ]

        # If camera is in colour mode and we are using colour, isolate the red channel
        if camera.get_camera_property()["IsColorCam"] and colour:
            calibrate_frames = [
                get_redness_frame(calibrated_frame) for calibrated_frame in calibrate_frames
            ]

        # Reset FSM position after frames captured
        fsm.send_command(f"{axis}=0", print_received= False)

        # Average all captured frames to reduce noise in the result
        calibrate_image = np.median(calibrate_frames, axis=0).astype(np.uint8)

        # Find contours and calculate the new position
        contours = get_contours(calibrate_image)
        largest_contour = get_largest_contour(contours)
        calibrated_pos_x, calibrated_pos_y = get_contour_origin(largest_contour)

        # Calculate how many pixels the dot moved
        delta_x = abs(origin_x - calibrated_pos_x)
        delta_y = abs(origin_y - calibrated_pos_y)
        assert delta_x != 0 and delta_y != 0, (
            f"Error: FSM did not move for the x and/or y axis. Check connection or signal."
        )
        angle = math.atan(delta_y / delta_x if axis == "x" else delta_x / delta_y)
        angles.append(angle)

    return tuple(angles)

def get_distance_to_camera(camera: asi.Camera,
                            master_dark: np.ndarray,
                            origin_pos: tuple[float, float],
                            fsm: FSM,
                            amplitude: float = 0.04,
                            num_frames: int = 10,
                            colour: bool = False) -> tuple[float, float]:

    fsm.send_command(f"xy=0;0", print_received= False)
    origin_x, origin_y = origin_pos

    assert -1.0 < amplitude < 1.0 and amplitude != 0, \
        "Amplitude must be within the FSM's limits (-1.0, 1.0) and not zero"

    # Move FSM to desired calibration position
    fsm.send_command(f"xy={amplitude};{amplitude}", print_received= False)

    # Capture and clean frames to find laser dot position
    calibrate_frames = [
        get_clean_frame(camera.capture(), master_dark) for _ in range(num_frames)
    ]

    # If camera is in colour mode and we are using colour, isolate the red channel
    if camera.get_camera_property()["IsColorCam"] and colour:
        calibrate_frames = [
            get_redness_frame(calibrated_frame) for calibrated_frame in calibrate_frames
        ]

    # Reset FSM position after frames captured
    fsm.send_command(f"xy=0;0", print_received= False)

    # Average all captured frames to reduce noise in the result
    calibrate_image = np.median(calibrate_frames, axis=0).astype(np.uint8)

    # Find contours and calculate the new position
    contours = get_contours(calibrate_image)
    largest_contour = get_largest_contour(contours)
    calibrated_pos_x, calibrated_pos_y = get_contour_origin(largest_contour)

    # Calculate how many pixels the dot moved
    delta_x = abs(origin_x - calibrated_pos_x)
    delta_y = abs(origin_y - calibrated_pos_y)

    angle = amplitude * math.tan(math.radians(50))
    distance_x = delta_x / math.tan(angle)
    distance_y = delta_y / math.tan(angle)

    assert delta_x != 0 and delta_y != 0, (
        f"Error: FSM did not move for the x and/or y axis. Check connection or signal."
    )

    return distance_x, distance_y

def get_damping_factor(current_error, previous_error) -> float:
    if (total_error := current_error + previous_error) == 0:
        return 1
    return current_error / total_error



