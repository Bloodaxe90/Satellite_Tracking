import math
import os
from collections import deque

import numpy as np
import pandas as pd
import time

from src.camera.camera_setup import setup_camera
from src.camera.camera_stream import CameraStream
from src.camera.image_processing import (
    get_clean_frame,
    get_contours,
    get_contour_origin,
    get_largest_contour,
)
from src.fsm.fsm_calibration import get_distance_to_camera, get_response_time
from src.fsm.fsm_setup import setup_fsm
from src.kalman_filter.kalman_filter_calibration import (
    is_sane_measurement,
    set_transition_matrix,
)
from src.utils.general import wait
from src.utils.io import save_results
from src.kalman_filter.kalman_filter_setup import setup_kalman_filter


def test():
    """
    Experimental test function for independent validation and debugging

    This function is not part of the core tracking mechanism and is primarily used
    for messy testing. It was originally written to:
      - Record noise characteristics from the camera with the FSM turned off
      - Run and retrieve results from linear tracking experiments without involving
        the FSM
      - Provide a quick environment to test new ideas

    Notes:
        - The function mixes setup, capture, filtering, and FSM logic in one place
          and is intentionally unstructured
        - Results are logged for later inspection
        - Should not be relied on for production use; serves only for
          testing new ideas or debugging issues
    """

    # General Parameters
    ROOT_DIR = os.path.dirname(os.getcwd())

    # Camera Parameters
    SDK_LIB_NAME: str = "libASICamera2.dylib"
    CAMERA_ID: int = 0
    BINS = 2
    GAIN = 120
    EXPOSURE = 32
    MAX_RESOLUTION = (int(8288 / BINS), int(5640 / BINS))
    RESOLUTION = MAX_RESOLUTION
    START_POS = (0, 0)
    FRAMES_DARK = 10

    # FSM Parameters
    PORT: str = "/dev/cu.usbmodem00000000001A1"
    BAUDRATE: int = 256000
    TIMEOUT: int = 1
    MAX_AMPLITUDE = 1
    MIN_AMPLITUDE = -1

    # Kalman Filter Parameters
    # TODO need tuning
    MODEL_UNCERTAINTY = 1e-6
    MEASUREMENT_UNCERTAINTY = 5
    LEARNING_ITERATIONS_KF = 300
    INSANE_THRESHOLD = 5

    # testing specific parameters (Including frequency plotter)
    ITERATIONS = 600
    KALMAN_FILTER = False
    FSM = False

    # Linear FSM
    FSM_LINEAR = True
    linear_amp_incr_x = 0.00005
    linear_amp_incr_y = 0.00005

    EXPERIMENT_NAME: str = (
        f"tracking_"
        f".04A_"
        f"KF{1 if KALMAN_FILTER else 0}_"
        f"G{GAIN}_"
        f"E{EXPOSURE}_"
        f"B{BINS}_"
        f"R{RESOLUTION}_"
        f"S{START_POS}_"
        f"FSM{1 if FSM else 0}_"
        f"MOU{MODEL_UNCERTAINTY}_"
        f"MEU{MEASUREMENT_UNCERTAINTY}_"
    )
    results = pd.DataFrame(
        columns=[
            "measured_X",
            "measured_Y",
            "estimated_X",
            "estimated_Y",
            "delta_x",
            "delta_y",
            "Time",
        ]
    )

    amplitude_bounds = (MIN_AMPLITUDE, MAX_AMPLITUDE, MIN_AMPLITUDE, MAX_AMPLITUDE)

    camera = None
    camera_stream = None
    fsm = None
    kalman_filter = None
    distance_x = None
    distance_y = None
    fsm_sleep_time = None

    try:
        camera, master_dark = setup_camera(
            root_dir=ROOT_DIR,
            sdk_lib_name=SDK_LIB_NAME,
            camera_id=CAMERA_ID,
            bins=BINS,
            gain=GAIN,
            exposure=EXPOSURE,
            resolution=RESOLUTION,
            start_pos=START_POS,
            dark_frames=FRAMES_DARK,
        )

        if FSM_LINEAR:
            fsm: FSM = setup_fsm(PORT, BAUDRATE, TIMEOUT)
            fsm.send_command(f"xy=0;0", False, False)
            time.sleep(1)

        if FSM:
            fsm: FSM = setup_fsm(PORT, BAUDRATE, TIMEOUT)

        wait("generate origin")
        origin_frames = [
            get_clean_frame(camera.capture(), master_dark) for _ in range(10)
        ]
        origin_image = np.median(origin_frames, axis=0).astype(np.uint8)
        contours = get_contours(origin_image)
        largest_contour = get_largest_contour(contours)
        origin_x, origin_y = get_contour_origin(largest_contour)

        if FSM:
            wait(
                "Ready to calibrate distance from camera.\n"
                "Please ensure the laser points to around the center of the camera"
            )

            distance_x, distance_y = get_distance_to_camera(
                camera=camera,
                master_dark=master_dark,
                origin_pos=(origin_x, origin_y),
                fsm=fsm,
                amplitude_bounds=amplitude_bounds,
            )

            print(f"Distance X{distance_x}, Y{distance_y}")

        camera_stream = CameraStream(camera).start()

        if FSM:
            fsm_sleep_time = get_response_time(
                camera_stream, master_dark, fsm, amplitude_bounds
            )

        if KALMAN_FILTER:
            initial_sample_time = 1 / camera_stream.get_update_rate()
            kalman_filter = setup_kalman_filter(
                delta_t=initial_sample_time,
                model_uncertainty=MODEL_UNCERTAINTY,
                measurement_uncertainty=MEASUREMENT_UNCERTAINTY,
            )

        wait("Start?")
        largest_contour_search_times = deque(maxlen=10)
        old_dropped_frames = 0
        amplitude_x = 0
        amplitude_y = 0
        start_time = time.time()
        last_time = start_time
        last_sleep_time = start_time
        insane_count = 0

        first_measurement = True

        i = 0
        while time.time() - start_time < 60:
            current_time = time.time()
            sample_time = current_time - last_time
            last_time = current_time
            if KALMAN_FILTER:
                set_transition_matrix(kalman_filter, sample_time)

                kalman_filter.predict()

            raw_frame = camera_stream.read()
            if raw_frame is None:
                time.sleep(0.001)
                continue

            clean_frame = get_clean_frame(raw_frame, master_dark)

            measured_x = None
            measured_y = None

            contours = get_contours(clean_frame)
            avg_largest_contour_search_time = (
                np.mean(largest_contour_search_times, axis=0) if i > 0 else 0
            )
            if contours:
                largest_contour_search_time = time.time()
                largest_contour = get_largest_contour(contours)
                measured_x, measured_y = get_contour_origin(largest_contour)
                largest_contour_search_times.append(
                    time.time() - largest_contour_search_time
                )
            else:
                time.sleep(
                    avg_largest_contour_search_time
                )  # to keep sample rate consistent for KF

            new_x = measured_x
            new_y = measured_y

            if KALMAN_FILTER:
                sane_measurement = (
                    is_sane_measurement(kalman_filter, measured_x, measured_y)
                    if i > LEARNING_ITERATIONS_KF
                    else True
                )  # Assumes the first 100 measurements are accurate to gather values for prediction

                if sane_measurement:
                    insane_count = 0
                else:
                    insane_count += 1

                if insane_count < INSANE_THRESHOLD:
                    if first_measurement:
                        first_measurement = False
                        kalman_filter.errorCovPost = np.eye(4, dtype=np.float32)
                        kalman_filter.statePost = np.array(
                            [[measured_x], [measured_y], [0], [0]], dtype=np.float32
                        )
                    else:
                        kalman_filter.correct(
                            np.array([[measured_x], [measured_y]], dtype=np.float32)
                        )
                else:
                    estimated_state = kalman_filter.statePost
                    new_x = estimated_state[0, 0]
                    new_y = estimated_state[1, 0]

                print(
                    f"M {measured_x} {measured_y}\nE {kalman_filter.statePost[0, 0]} {kalman_filter.statePost[1, 0]}\nSane {sane_measurement}"
                )

            delta_x = origin_x - new_x
            delta_y = origin_y - new_y

            results.loc[len(results)] = {
                "measured_X": measured_x,
                "measured_Y": measured_y,
                "estimated_X": (
                    kalman_filter.statePost[0, 0] if KALMAN_FILTER else new_x
                ),
                "estimated_Y": (
                    kalman_filter.statePost[1, 0] if KALMAN_FILTER else new_y
                ),
                "delta_x": delta_x,
                "delta_y": delta_y,
                "Time": time.time() - start_time,
            }

            if (i + 1) % 100 == 0:
                new_dropped_frames = camera.get_dropped_frames()
                print(
                    f"Iteration: {i + 1}, Update Rate: {camera_stream.get_update_rate()}, Sample Rate: {(i + 1) / (time.time() - start_time)}, Dropped Frames: {new_dropped_frames - old_dropped_frames}"
                )
                old_dropped_frames = new_dropped_frames

            if FSM:
                current_sleep_time = time.time()
                if current_sleep_time - last_sleep_time >= fsm_sleep_time:
                    amplitude_x += delta_x / (distance_x * math.tan(math.radians(50)))
                    amplitude_y -= delta_y / (distance_y * math.tan(math.radians(50)))

                    assert (
                        MIN_AMPLITUDE < amplitude_x < MAX_AMPLITUDE
                        and MIN_AMPLITUDE < amplitude_y < MAX_AMPLITUDE
                    ), f"Amplitude must be within the FSM's limits {amplitude_bounds}"
                    fsm.send_command(
                        f"xy={amplitude_x};{amplitude_y}",
                        print_sent=False,
                        print_received=False,
                    )
                    last_sleep_time = current_sleep_time

            if FSM_LINEAR:
                amplitude_x += linear_amp_incr_x
                amplitude_y += linear_amp_incr_y
                fsm.send_command(f"xy={amplitude_x};{amplitude_y}", False, False)

            i += 1

    except (KeyboardInterrupt, RuntimeError, AssertionError) as e:
        print(f"\nTracking stopped due to Error: {e}")

    finally:
        print("Cleaning up")
        if fsm is not None:
            fsm.send_command("xy=0;0")
            fsm.disconnect()
        if camera_stream is not None:
            camera_stream.stop()
        if camera is not None:
            camera.close()
        save_results(ROOT_DIR, results, f"{EXPERIMENT_NAME}")
        print("Cleanup complete.")


if __name__ == "__main__":
    test()
