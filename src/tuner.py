import math
import os
from collections import deque

import numpy as np
import pandas as pd
import time

from src.camera.camera_setup import setup_camera
from src.camera.camera_stream import CameraStream
from src.camera.image_processing import get_clean_frame, \
     get_contours, get_contour_origin, get_largest_contour
from src.fsm.fsm_calibration import get_distance_to_camera
from src.fsm.fsm_setup import setup_fsm
from src.kalman_filter.kalman_filter_calibration import is_sane_measurement, \
    set_transition_matrix
from src.utils.general import wait
from src.utils.io import save_results
from src.kalman_filter.kalman_filter_setup import setup_kalman_filter


def tuner():
    # Tuner assumes there will always be a measurement and that all measurements are valid and can be given to the Kalman filter

    # General Parameters
    ROOT_DIR = os.path.dirname(os.getcwd())
    KERNEL_SIZE = 3

    # Camera Parameters
    SDK_LIB_NAME: str = 'libASICamera2.dylib'
    CAMERA_ID: int = 0
    BINS = 2
    GAIN = 120
    EXPOSURE = 32
    RESOLUTION = (8288, 5640)
    START_POS = (0, 0)
    FRAMES_DARK = 10

    # FSM Parameters
    PORT: str = "/dev/cu.usbmodem00000000001A1"
    BAUDRATE: int = 256000
    TIMEOUT: int = 1

    # Kalman Filter Tuner
    SEARCH_ITERATIONS = 4
    INITIAL_MODEL_UNCERTAINTIES = [1e-6, 1e-5, 1e-4, 1e-3, 1e-2]
    INITIAL_MEASUREMENT_UNCERTAINTIES = [0.01, 0.1, 1.0, 10.0, 100.0]
    NUM_MODEL_UNCERTAINIES = 5
    NUM_MEASUREMENT_UNCERTAINIES = 5
    MODEL_REDUCTION_FACTOR = 0.8
    MEASUREMENT_REDUCTION_FACTOR = 0.5

    # testing specific parameters (Including frequency plotter)
    ITERATIONS = 200
    FSM = False

    # Linear FSM
    FSM_LINEAR = True
    initial_linear_amp_x = -0.04
    initial_linear_amp_y = -0.04
    linear_amp_incr_x = 0.00005
    linear_amp_incr_y = 0.00005

    EXPERIMENT_NAME: str = (f"tuner3_"
                            f".04A_"
                            f"SI{SEARCH_ITERATIONS}_"
                            f"I{ITERATIONS}_"
                            f"G{GAIN}_"
                            f"E{EXPOSURE}_"
                            f"B{BINS}_"
                            f"R{RESOLUTION}_"
                            f"S{START_POS}_"
                            f"FSM{1 if FSM else 0}_"
                            f"K{KERNEL_SIZE}")

    final_results = pd.DataFrame(columns=[
        "model_uncertainty",
        "measurement_uncertainty",
        "error_x",
        "error_y",
    ])

    camera = None
    camera_stream = None
    fsm = None
    distance_x = None
    distance_y = None

    try:
        camera, master_dark = setup_camera(root_dir=ROOT_DIR,
                                           sdk_lib_name=SDK_LIB_NAME,
                                           camera_id=CAMERA_ID,
                                           bins=BINS,
                                           gain=GAIN,
                                           exposure=EXPOSURE,
                                           resolution=RESOLUTION,
                                           start_pos=START_POS,
                                           num_frames=FRAMES_DARK,)

        if FSM_LINEAR:
            fsm: FSM = setup_fsm(PORT, BAUDRATE, TIMEOUT)
            fsm.send_command(f"xy={initial_linear_amp_x};{initial_linear_amp_y}", False, False)
            time.sleep(1)

        if FSM:
            fsm: FSM = setup_fsm(PORT, BAUDRATE, TIMEOUT)

        origin_x, origin_y = ((RESOLUTION[0] / BINS) / 2, (RESOLUTION[1] / BINS) / 2)

        #TODO remove later
        origin_frames = [get_clean_frame(camera.capture(), master_dark) for
                            _ in range(10)]
        origin_image = np.median(origin_frames, axis=0).astype(np.uint8)
        contours = get_contours(origin_image)
        largest_contour = get_largest_contour(contours)
        origin_x, origin_y = get_contour_origin(largest_contour)


        if FSM:
            wait("Ready to calibrate distance from camera.\n"
                 "Please ensure the laser points to around the center of the camera")

            distance_x, distance_y = get_distance_to_camera(camera=camera,
                master_dark=master_dark,
                origin_pos=(origin_x, origin_y),
                fsm=fsm)

            print(f"Distance X {distance_x}, Y {distance_y}")

        camera_stream = CameraStream(camera).start()
        wait("Start?")

        model_uncertainties = INITIAL_MODEL_UNCERTAINTIES
        measurement_uncertainties = INITIAL_MEASUREMENT_UNCERTAINTIES
        for i in range(SEARCH_ITERATIONS):

            print(f"{i + 1}. MOU {model_uncertainties}, MEU {measurement_uncertainties}")
            for model_uncertainty in model_uncertainties:
                for measurement_uncertainty in measurement_uncertainties:

                    print(f"Tested parameters: MOU {model_uncertainty}, MEU {measurement_uncertainty}")
                    error_x = []
                    error_y = []

                    linear_amp_x = initial_linear_amp_x
                    linear_amp_y = initial_linear_amp_y

                    if FSM_LINEAR:
                        fsm.send_command(f"xy={linear_amp_x};{linear_amp_y}",
                                         False, False)
                        time.sleep(1)

                    initial_sample_time = 1 / camera_stream.get_update_rate()
                    kalman_filter = setup_kalman_filter(delta_t=initial_sample_time,
                                                        model_uncertainty=model_uncertainty,
                                                        measurement_uncertainty=measurement_uncertainty)
                    old_delta_x = 0
                    old_delta_y = 0
                    start_time = time.time()
                    last_time = start_time
                    first_measurement = True
                    for i in range(ITERATIONS):

                        current_time = time.time()
                        sample_time = current_time - last_time
                        last_time = current_time
                        set_transition_matrix(kalman_filter, sample_time)

                        kalman_filter.predict()

                        raw_frame = camera_stream.read()
                        if raw_frame is None:
                            time.sleep(0.001)
                            continue

                        clean_frame = get_clean_frame(raw_frame, master_dark, KERNEL_SIZE)

                        contours = get_contours(clean_frame)
                        largest_contour = get_largest_contour(contours)
                        measured_x, measured_y = get_contour_origin(largest_contour)

                        sane_measurement = is_sane_measurement(
                            kalman_filter, measured_x, measured_y
                        )

                        if first_measurement:
                            first_measurement = False
                            kalman_filter.errorCovPost = np.eye(4, dtype=np.float32)
                            kalman_filter.statePost = np.array(
                                [[measured_x], [measured_y], [0], [0]], dtype=np.float32)
                        else:
                            kalman_filter.correct(np.array([[measured_x], [measured_y]], dtype=np.float32))
                        estimated_state = kalman_filter.statePost
                        estimated_x = estimated_state[0, 0]
                        estimated_y = estimated_state[1, 0]

                        #print(f"M {measured_x} {measured_y}\nE {estimated_x} {estimated_y}\nSane {sane_measurement}")

                        delta_x = origin_x - measured_x
                        delta_y = origin_y - measured_y

                        error_x.append(abs(measured_x - estimated_x))
                        error_y.append(abs(measured_y - estimated_y))

                        #print(f"Error X {error_x[-1]} Y {error_y[-1]}")

                        # if (i + 1) % 100 == 0:
                        #     print(f"Iteration: {i + 1}, FPS: {camera_stream.get_fps()}, Sample Rate: {(i + 1) / (time.time() -  start_time)}")

                        if FSM:
                            new_amplitude_x = delta_x / (distance_x * math.tan(math.radians(50)))
                            new_amplitude_y = delta_y / (distance_y * math.tan(math.radians(50)))

                            delta_x = abs(delta_x)
                            delta_y = abs(delta_y)

                            damping_factor_x = get_damping_factor(delta_x, old_delta_x)
                            damping_factor_y = get_damping_factor(delta_y, old_delta_y)

                            amplitude_x = new_amplitude_x
                            amplitude_y = new_amplitude_y
                            assert -1.0 <= amplitude_x <= 1.0 and -1.0 <= amplitude_y <= 1.0, f"Amplitudes ({amplitude_x}, {amplitude_y}) must be between -1.0 and 1.0"
                            fsm.send_command(f"xy={(amplitude_x * damping_factor_x)};{amplitude_y * damping_factor_y}", print_sent= False, print_received= False)

                            old_delta_x = delta_x
                            old_delta_y = delta_y

                        if FSM_LINEAR:
                            linear_amp_x += linear_amp_incr_x
                            linear_amp_y += linear_amp_incr_y
                            fsm.send_command(f"xy={linear_amp_x};{linear_amp_y}", False, False)

                    final_results.loc[len(final_results)] = {"model_uncertainty" : model_uncertainty,
                                                "measurement_uncertainty" : measurement_uncertainty,
                                                "error_x" : np.mean(np.array(error_x), axis = 0),
                                                "error_y" : np.mean(np.array(error_y), axis = 0),}

            rmse = np.sqrt(final_results['error_x']**2 + final_results['error_y']**2)
            best_index = np.argmin(rmse, axis = 0)

            center_value = final_results["model_uncertainty"][best_index]
            half_range = center_value * MODEL_REDUCTION_FACTOR
            new_space = np.linspace(center_value - half_range,
                                    center_value + half_range, NUM_MODEL_UNCERTAINIES)
            model_uncertainties = np.array([max(x, 1e-9) for x in new_space]).astype(np.float32)


            center_value = final_results["measurement_uncertainty"][best_index]
            half_range = center_value * MEASUREMENT_REDUCTION_FACTOR
            new_space = np.linspace(center_value - half_range,
                                    center_value + half_range, NUM_MEASUREMENT_UNCERTAINIES)

            measurement_uncertainties = np.array([max(x, 1e-9) for x in new_space]).astype(np.float32)



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
        save_results(ROOT_DIR, final_results, f"{EXPERIMENT_NAME}")
        print("Cleanup complete.")

if __name__ == "__main__":
    tuner()