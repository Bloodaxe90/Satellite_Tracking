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
    set_transition_matrix
)
from src.utils.general import wait
from src.utils.io import save_results
from src.kalman_filter.kalman_filter_setup import setup_kalman_filter


def test():
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
    MEASUREMENT_UNCERTAINTY = 30
    LEARNING_ITERATIONS_KF = 100
    INSANE_THRESHOLD = 1

    # testing specific parameters (Including frequency plotter)
    ITERATIONS = 1000
    TIME = 300
    KALMAN_FILTER = False
    FSM = False

    # Linear FSM
    FSM_LINEAR = False
    initial_amplitude_x = -0.04
    initial_amplitude_y = -0.04
    scaling_factor = TIME / 60
    linear_amp_incr_x = 0.00001 / scaling_factor
    linear_amp_incr_y = 0.00001 / scaling_factor
    amp_x_acc = 1.005 ** (1 / scaling_factor)
    amp_y_acc = 1.005 ** (1 / scaling_factor)
    LEARNING_ITERATIONS_KF *= scaling_factor
    amp_x_dec = 2 - amp_x_acc
    amp_y_dec = 2 - amp_y_acc

    EXPERIMENT_NAME: str = (
        f"ca_tracking_"
        f".003A_"
        f"T{TIME}"
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

    MODEL_UNCERTAINTY = 5e-4
    MEASUREMENT_UNCERTAINTY = 1000

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
            fsm.send_command(f"xy={initial_amplitude_x};{initial_amplitude_y}", False, False)
            time.sleep(1)

        if FSM:
            fsm: FSM = setup_fsm(PORT, BAUDRATE, TIMEOUT)

        wait("generate origin")
        origin_frames = [
            get_clean_frame(camera.capture(), master_dark)
            for _ in range(10)
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
        amplitude_x = initial_amplitude_x
        amplitude_y = initial_amplitude_y
        start_time = time.time()
        last_time = start_time
        last_sleep_time = start_time
        insane_count = 0

        first_measurement = True
        changed = False

        i = 0
        while time.time() - start_time < TIME:
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


            if (amplitude_x > 0 or amplitude_y > 0) and not changed:
                changed = True
                F = np.eye(8, dtype=np.float32)
                F[4, 4] = -1.0  # Flip ax
                F[5, 5] = -1.0  # Flip ay
                # F[6, 6] = -1.0  # Flip jx
                # F[7, 7] = -1.0  # Flip jy

                kalman_filter.statePost = F @ kalman_filter.statePost

                kalman_filter.errorCovPost = F @ kalman_filter.errorCovPost @ F.T

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
                        kalman_filter.errorCovPost = np.eye(8,
                                                            dtype=np.float32) * 500.0

                        kalman_filter.statePost = np.array([
                            [measured_x],  # Position x
                            [measured_y],  # Position y
                            [0],  # Velocity x
                            [0],  # Velocity y
                            [0],  # Acceleration x
                            [0],  # Acceleration y
                            [0],  # Jerk x
                            [0]  # Jerk y
                        ], dtype=np.float32)
                    else:
                        kalman_filter.correct(
                            np.array([[measured_x], [measured_y]],
                                     dtype=np.float32)
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
                linear_amp_incr_x *= (amp_x_acc if amplitude_x < 0 and amplitude_y < 0 else amp_x_dec)
                linear_amp_incr_y *= (amp_y_acc if amplitude_x < 0 and amplitude_y < 0 else amp_y_dec)
                print(linear_amp_incr_x, linear_amp_incr_y, amplitude_x, amplitude_y)
                fsm.send_command(f"xy={amplitude_x};{amplitude_y}", False, False)

            i += 1

        print("Iterations:", i)
        print("Runtime", time.time() - start_time)

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
