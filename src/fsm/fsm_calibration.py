import math
import time
from typing import Tuple, Any

import cv2
import numpy as np
import zwoasi as asi

from src.camera.camera_stream import CameraStream
from src.camera.image_processing import get_redness_frame, get_clean_frame, \
    get_contours, get_contour_origin, get_largest_contour
from src.fsm.fsm import FSM


def get_fsm_roi(camera: asi.Camera,
                master_dark: np.ndarray,
                fsm: FSM,
                amplitude_incr: float = 0.001,
                ) -> tuple[tuple[float, float] | Any, ...]:
    #TODO add kalman filter find definite drop off point
    bounds = []

    for idx, axis in enumerate(["x", "y"]):
        for _ in range(2):
            fsm.send_command(f"xy=0;0", print_received=False)
            bound = None
            amplitude = 0
            amplitude_incr = amplitude_incr * -1

            while True:
                if amplitude > 1 or amplitude < -1:
                    break
                # Move FSM to desired calibration position
                fsm.send_command(f"{axis}={amplitude}", print_received=False)

                # Capture and clean frame to find laser dot position
                calibrate_image = get_clean_frame(camera.capture(),
                                                  master_dark)

                # Find contours and calculate the new position
                contours = get_contours(calibrate_image)
                if not contours:
                    break

                largest_contour = get_largest_contour(contours)
                bound = get_contour_origin(largest_contour)
                print(f"Pos {bound}")

                amplitude += amplitude_incr

            bounds.append(bound)

    return tuple(bounds)

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

    return abs(distance_x), abs(distance_y)

def get_response_time(camera_stream: CameraStream,
                      master_dark: np.ndarray,
                      fsm: FSM,
                      kernel_size: int,
                      measurements: int = 100,
                      max_amplitude: float = -0.02,
                      threshold: int = 5,
                      ):
    amplitude = 0
    last_measured_x = 0
    last_measured_y = 0
    first_iteration = True
    last_time = time.time()
    times = []
    while len(times) < measurements:

        fsm.send_command(f"xy={amplitude};{amplitude}", False, False)

        raw_frame = camera_stream.read()
        if raw_frame is None:
            time.sleep(0.001)
            continue

        clean_frame = get_clean_frame(raw_frame, master_dark, kernel_size)

        contours = get_contours(clean_frame)
        largest_contour = get_largest_contour(contours)
        measured_x, measured_y = get_contour_origin(largest_contour)

        if first_iteration:
            first_iteration = False
            last_time = time.time()
        else:
            delta_x = abs(measured_x - last_measured_x)
            delta_y = abs(measured_y - last_measured_y)

            if delta_x > threshold and delta_y > threshold:
                current_time = time.time()
                times.append(current_time - last_time)
                last_time = current_time

        amplitude = max_amplitude if amplitude == 0 else 0

        last_measured_x = measured_x
        last_measured_y = measured_y
    fsm.send_command(f"xy=0;0", False, False)

    avg_response_time = np.mean(times, axis = 0)
    print(f"Average response time {avg_response_time}")
    return avg_response_time












