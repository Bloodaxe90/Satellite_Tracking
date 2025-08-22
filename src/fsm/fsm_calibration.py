import math
import time
from collections import deque
import numpy as np
import zwoasi as asi

from src.camera.camera_stream import CameraStream
from src.camera.image_processing import (
    get_clean_frame,
    get_contours,
    get_contour_origin,
    get_largest_contour,
    get_stacked_frame,
)
from src.fsm.fsm import FSM


def get_alignment_offset_angle(
    camera: asi.Camera,
    master_dark: np.ndarray,
    origin_pos: tuple[float, float],
    fsm: FSM,
    amplitude_bounds: tuple,
    amplitude: float = 0.04,
    num_frames: int = 10,
) -> tuple[float, ...]:
    """
    Determines the angular offset of the fsm and the camera by moving the mirror along
    each axis, capturing laser dot positions, and calculating relative shifts in the other axis.

    Args:
        camera (asi.Camera): camera used for image capture
        master_dark (np.ndarray): Master dark frame for noise reduction
        origin_pos (tuple[float, float]): Original laser dot (x, y) position in pixels
        fsm (FSM): FSM for sending axis movement commands
        amplitude_bounds (tuple): The maximum and minimum amplitudes for each axis in each direction
        amplitude (float, optional): Movement amplitude for FSM axis calibration (default: 0.04)
        num_frames (int, optional): Number of frames to stack across (default: 10)

    Returns:
        tuple[float, ...]: Calculated offset angles (x_angle, y_angle) in radians
    """

    print("Getting offset alignment angle")
    # Reset FSM to center position
    fsm.send_command("xy=0;0", print_received=False)
    origin_x, origin_y = origin_pos

    min_amplitude_x, max_amplitude_x, min_amplitude_y, max_amplitude_y = (
        amplitude_bounds
    )

    assert (
        min_amplitude_x < amplitude < max_amplitude_x
        and min_amplitude_y < amplitude < max_amplitude_y
        and amplitude != 0
    ), f"Amplitude must be within the FSM's limits {amplitude_bounds} and not zero"

    angles = []

    for axis in ["x", "y"]:

        # Move FSM by amplitude for calibration
        fsm.send_command(f"{axis}={amplitude}", print_received=False)

        # Capture and clean multiple frames
        calibrate_frames = [
            get_clean_frame(camera.capture(), master_dark) for _ in range(num_frames)
        ]

        # Reset FSM axis back to origin after data capture
        fsm.send_command(f"{axis}=0", print_received=False)

        # stack frames to reduce noise
        calibrate_image = get_stacked_frame(calibrate_frames)

        # Find the laser dot position from contours
        contours = get_contours(calibrate_image)
        largest_contour = get_largest_contour(contours)
        calibrated_pos_x, calibrated_pos_y = get_contour_origin(largest_contour)

        # Calculate displacement of laser dot
        delta_x = abs(origin_x - calibrated_pos_x)
        delta_y = abs(origin_y - calibrated_pos_y)

        # Ensure FSM movement was detected
        assert (
            delta_x != 0 and delta_y != 0
        ), f"Error: FSM did not move for the {axis}-axis. Check connection or signal."

        # Compute angle depending on which axis was moved
        angle = math.atan(delta_y / delta_x if axis == "x" else delta_x / delta_y)
        angles.append(angle)
    print(f"Offset alignments angles: X {angles[0]}, Y{angles[1]}\n")

    return tuple(angles)


def get_fsm_roi_and_amplitude_bounds(
    camera: asi.Camera,
    master_dark: np.ndarray,
    fsm: FSM,
    max_resolution: tuple[int, int],
    padding: int = 20,
    amplitude_incr: float = 0.001,
    learning_iterations: int = 10,
    std_devs: float = 5,
) -> tuple:
    """
    Determines the region of interest (ROI) and amplitude bounds for the FSM.
    The FSM is moved incrementally along both axes until the laser dot cannot be seen

    Args:
        camera (asi.Camera): camera used for capturing frames
        master_dark (np.ndarray): Master dark frame used for noise reduction
        fsm (FSM): FSM for sending movement commands
        max_resolution (tuple[int, int]): Maximum resolution (width, height) of the camera
        padding (int, optional): Extra padding around detected ROI in pixels (default: 20)
        amplitude_incr (float, optional): Increment/decrement step size for FSM movement (default=0.001)
        learning_iterations (int, optional): Number of iterations used to learn baseline movement distances (default=10)
        std_devs (float, optional): Number of standard deviations above baseline to define movement threshold (default=5)

    Returns:
        tuple: (start_pos, resolution, amplitudes)
            start_pos (tuple[int, int]): Top left starting (x, y) coordinates of the ROI
            resolution (tuple[int, int]): Width and height of the ROI adjusted to hardware constraints
            amplitudes (list[float]): Final amplitude values reached for both axes
    """

    bounds = []
    amplitudes = []

    print(f"Calibrating fsm roi and amplitude bounds")

    for idx, axis in enumerate(["x", "y"]):
        for _ in range(2):
            last_position = None
            position = None
            first_iteration = True
            distances_moved = deque(maxlen=learning_iterations)
            amplitude = 0
            amplitude_incr = amplitude_incr * -1  # change scan direction

            i = 0
            while True:
                if first_iteration:
                    # Reset FSM
                    first_iteration = False
                    fsm.send_command("xy=0;0", print_received=False)
                else:
                    # Stop scanning if amplitude moves outside FSM limits
                    if amplitude > 1 or amplitude < -1:
                        break

                    # Move FSM by amplitude along the chosen axis
                    fsm.send_command(
                        f"{axis}={amplitude}", print_sent=False, print_received=False
                    )

                time.sleep(0.1)  # Allow FSM movement to settle

                # Capture frame and subtract dark frame
                calibrate_image = get_clean_frame(camera.capture(), master_dark)

                # Extract contours to locate laser dot
                contours = get_contours(calibrate_image)
                if not contours:
                    break  # Stop if laser is not detected

                # Find the largest contour
                largest_contour = get_largest_contour(contours)
                position = get_contour_origin(largest_contour)

                if last_position:
                    # Measure displacement of laser dot along the scanned axis
                    distance_moved = np.linalg.norm(position[idx] - last_position[idx])

                    # Build up baseline movement
                    if i < learning_iterations:
                        distances_moved.append(distance_moved)
                    else:
                        # Use baseline movement to detect boundry
                        mean_distance_moved = np.mean(distances_moved, axis=0)
                        std_dev_distance_moved = np.std(distances_moved, axis=0)
                        threshold = mean_distance_moved + (
                            std_devs * std_dev_distance_moved
                        )

                        # If displacement exceeds threshold then the boundary is reached
                        if distance_moved > threshold:
                            break

                amplitude += amplitude_incr
                last_position = position
                i += 1

            amplitudes.append(amplitude)

            # Ensure laser position was successfully found
            assert position is not None, (
                "Initial Position cannot be found, please ensure laser is on and in frame "
                "(check resources/images/test_image.jpg)"
            )
            bounds.append(position[idx])

    min_x, max_x, max_y, min_y = bounds
    max_width, max_height = max_resolution

    # Expand ROI slightly using padding and then align to hardware constraints
    final_width = int((np.ceil((max_x - min_x) + (padding * 2)) / 8.0)) * 8
    final_height = int((np.ceil((max_y - min_y) + (padding * 2)) / 2.0)) * 2

    start_x = max(0, int(min_x - padding))
    start_y = max(0, int(min_y - padding))

    # Ensure ROI does not exceed the maximum resolution
    if start_x + final_width > max_width:
        start_x = max_width - final_width
    if start_y + final_height > max_height:
        start_y = max_height - final_height

    resolution = (final_width, final_height)
    start_pos = (start_x, start_y)

    # Reset FSM to center position before finishing
    fsm.send_command("xy=0;0", print_received=False)

    print(
        f"Original Bounds Positions: min X {min_x}, max X {max_x}, min Y {min_y}, max Y {max_y}"
        f"Starting Position: {start_pos}"
        f"Resolution: {resolution}"
        f"Amplitude Bounds min X {amplitudes[0]}, max X {amplitudes[1]}, min Y {amplitudes[2]}, max Y {amplitudes[3]}\n"
    )

    return start_pos, resolution, amplitudes


def get_distance_to_camera(
    camera: asi.Camera,
    master_dark: np.ndarray,
    origin_pos: tuple[float, float],
    fsm: FSM,
    amplitude_bounds: tuple,
    amplitude: float = 0.04,
    num_frames: int = 10,
) -> tuple[float, float]:
    """
    Estimates the distance from the camera to the fsm in both X and Y axes.
    The FSM is moved diagonally by an amplitude, and
    the resulting shift in the laser dot is measured. Using trig the
    displacement is converted into a distance estimate.

    Args:
        camera (asi.Camera): camera used for capturing frames
        master_dark (np.ndarray): Master dark frame used for noise reduction
        origin_pos (tuple[float, float]): Initial (x, y) pixel coordinates of the laser
        fsm (FSM): FSM for sending movement commands
        amplitude_bounds (tuple): The maximum and minimum amplitudes for each axis in each direction
        amplitude (float, optional): Normalized FSM displacement in both axes (default: 0.04).
        num_frames (int, optional): Number of frames to stack across (default: 10)

    Returns:
        tuple[float, float]: Estimated distances to the camera along (x, y) axes in pixels.
    """

    print("Calibrating FSMs distance from camera")
    # Reset FSM
    fsm.send_command("xy=0;0", print_received=False)
    origin_x, origin_y = origin_pos

    min_amplitude_x, max_amplitude_x, min_amplitude_y, max_amplitude_y = (
        amplitude_bounds
    )

    assert (
        min_amplitude_x < amplitude < max_amplitude_x
        and min_amplitude_y < amplitude < max_amplitude_y
        and amplitude != 0
    ), f"Amplitude must be within the FSM's limits {amplitude_bounds} and not zero"

    # Move FSM diagonally
    fsm.send_command(f"xy={amplitude};{amplitude}", print_received=False)

    # Capture and clean frames
    calibrate_frames = [
        get_clean_frame(camera.capture(), master_dark) for _ in range(num_frames)
    ]

    fsm.send_command("xy=0;0", print_received=False)

    # Stack frames to reduce noise
    calibrate_image = get_stacked_frame(calibrate_frames)

    # Find laser position
    contours = get_contours(calibrate_image)
    largest_contour = get_largest_contour(contours)
    calibrated_pos_x, calibrated_pos_y = get_contour_origin(largest_contour)

    # Measure displacement in pixels
    delta_x = abs(origin_x - calibrated_pos_x)
    delta_y = abs(origin_y - calibrated_pos_y)

    # Convert displacement to distance using trig
    angle = amplitude * math.tan(
        math.radians(50)
    )  # Calibration angle factor found in FSM manual
    distance_x = delta_x / math.tan(angle)
    distance_y = delta_y / math.tan(angle)

    distances = (abs(distance_x), abs(distance_y))

    print(f"Distances: X {distances[0]}, Y {distances[1]}\n")

    return distances


def get_response_time(
    camera_stream: CameraStream,
    master_dark: np.ndarray,
    fsm: FSM,
    amplitude_bounds: tuple,
    measurements: int = 100,
    max_amplitude: float = -0.02,
    threshold: int = 5,
) -> float:
    """
    Measure the average response time between sending a command to the fsm and
    observing the outcome of the command in the cameras stream

    Args:
        camera_stream (CameraStream): camera stream for reading frames
        master_dark (np.ndarray): Master dark frame used for noise reduction
        fsm (FSM): FSM used to send movement commands
        amplitude_bounds (tuple): The maximum and minimum amplitudes for each axis in each direction
        measurements (int, optional): Number of response time measurements to collect (default: 100)
        max_amplitude (float, optional): FSM movement amplitude (default: -0.02).
        threshold (int, optional): Minimum pixel displacement in both axes to count as a valid movement (default: 5)

    Returns:
        float: Average FSM response time in seconds across all measurements
    """

    amplitude = 0
    last_measured_x = 0
    last_measured_y = 0
    first_iteration = True
    last_time = time.time()
    times = []

    min_amplitude_x, max_amplitude_x, min_amplitude_y, max_amplitude_y = (
        amplitude_bounds
    )

    assert (
        min_amplitude_x < amplitude < max_amplitude_x
        and min_amplitude_y < amplitude < max_amplitude_y
        and amplitude != 0
    ), f"Amplitude must be within the FSM's limits {amplitude_bounds} and not zero"

    print("Calibrating Cameras response time to FSM")

    while len(times) < measurements:

        # Move FSM between origin and max_amplitude
        fsm.send_command(f"xy={amplitude};{amplitude}", False, False)

        raw_frame = camera_stream.read()
        if raw_frame is None:
            time.sleep(0.001)
            continue

        # Clean frame
        clean_frame = get_clean_frame(raw_frame, master_dark)

        # Find laser position
        contours = get_contours(clean_frame)
        largest_contour = get_largest_contour(contours)
        measured_x, measured_y = get_contour_origin(largest_contour)

        if first_iteration:
            first_iteration = False
            last_time = time.time()
        else:
            # Calculate pixel displacement relative to last position
            delta_x = abs(measured_x - last_measured_x)
            delta_y = abs(measured_y - last_measured_y)

            # If laser dot moved more than threshold take note of response time
            if delta_x > threshold and delta_y > threshold:
                current_time = time.time()
                times.append(current_time - last_time)
                last_time = current_time

        # Change amplitude for next iteration
        amplitude = max_amplitude if amplitude == 0 else 0

        last_measured_x = measured_x
        last_measured_y = measured_y

    # Reset FSM
    fsm.send_command("xy=0;0", False, False)

    # Compute average response time
    avg_response_time = np.mean(times, axis=0)

    print(f"Average response time {avg_response_time:.6f} seconds\n")
    return avg_response_time
