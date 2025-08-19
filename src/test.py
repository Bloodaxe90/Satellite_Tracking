import math
import os
from collections import deque

import numpy as np
import pandas as pd
import time

from src.camera.camera_calibration import set_roi, get_master_dark
from src.camera.camera_setup import setup_camera
from src.camera.camera_stream import CameraStream
from src.camera.image_processing import get_clean_frame, \
     get_contours, get_contour_origin, get_largest_contour
from src.fsm.fsm_calibration import get_distance_to_camera, \
    get_fsm_roi, get_response_time
from src.fsm.fsm_setup import setup_fsm
from src.kalman_filter.kalman_filter_calibration import is_sane_measurement, \
    set_transition_matrix
from src.utils.general import wait
from src.utils.io import save_results
from src.kalman_filter.kalman_filter_setup import setup_kalman_filter


def test():
    # General Parameters
    ROOT_DIR = os.path.dirname(os.getcwd())
    KERNEL_SIZE = 3

    # Camera Parameters
    SDK_LIB_NAME: str = 'libASICamera2.dylib'
    CAMERA_ID: int = 0
    BINS = 2
    GAIN = 120
    EXPOSURE = 32
    RESOLUTION = (3008, 2820) # MAX (8288, 5640) for bin 1
    START_POS = (1136, 0)
    FRAMES_DARK = 10

    # FSM Parameters
    PORT: str = "/dev/cu.usbmodem00000000001A1"
    BAUDRATE: int = 256000
    TIMEOUT: int = 1

    # Kalman Filter Parameters
    #TODO need tuning
    MODEL_UNCERTAINTY = 0.058320
    MEASUREMENT_UNCERTAINTY = 0.001250

    # testing specific parameters (Including frequency plotter)
    ITERATIONS = 1000
    KALMAN_FILTER = False
    FSM = False

    # Linear FSM
    FSM_LINEAR = False
    linear_amp_x = -0.04
    linear_amp_y = -0.04
    linear_amp_incr_x = 0.00005
    linear_amp_incr_y = 0.00005

    EXPERIMENT_NAME: str = (f"noise_reduction_sine_"
                            f".001A_"
                            f"KF{1 if KALMAN_FILTER else 0}_"
                            f"G{GAIN}_"
                            f"E{EXPOSURE}_"
                            f"B{BINS}_"
                            f"R{RESOLUTION}_"
                            f"S{START_POS}_"
                            f"FSM{1 if FSM else 0}_"
                            f"MOU{MODEL_UNCERTAINTY}_"
                            f"MEU{MEASUREMENT_UNCERTAINTY}_"
                            f"K{KERNEL_SIZE}")
    results = pd.DataFrame(columns=[
        "measured_X",
        "measured_Y",
        "estimated_X",
        "estimated_Y",
        "delta_x",
        "delta_y",
        "Time"
    ])

    camera = None
    camera_stream = None
    fsm = None
    kalman_filter = None
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
            fsm.send_command(f"xy={linear_amp_x};{linear_amp_y}", False, False)
            time.sleep(1)

        if FSM:
            fsm: FSM = setup_fsm(PORT, BAUDRATE, TIMEOUT)

        origin_x, origin_y = ((RESOLUTION[0] / BINS) / 2, (RESOLUTION[1] / BINS) / 2)

        #TODO remove later
        wait("generate origin")
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
                fsm=fsm,)

            print(f"Distance X{distance_x}, Y{distance_y}")

        camera_stream = CameraStream(camera).start()

        if FSM:
            fsm_sleep_time = get_response_time(camera_stream, master_dark, fsm, KERNEL_SIZE)

        if KALMAN_FILTER:
            initial_sample_time = 1 / camera_stream.get_update_rate()
            kalman_filter = setup_kalman_filter(delta_t=initial_sample_time,
                                                model_uncertainty=MODEL_UNCERTAINTY,
                                                measurement_uncertainty=MEASUREMENT_UNCERTAINTY)

        wait("Start?")
        largest_contour_search_times = deque(maxlen=10)
        old_dropped_frames = 0
        amplitude_x = 0
        amplitude_y = 0
        start_time = time.time()
        last_time = start_time
        last_sleep_time = start_time

        first_measurement = True

        i = 0
        while time.time() - start_time < 60:
        # for i in range(ITERATIONS):
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

            clean_frame = get_clean_frame(raw_frame, master_dark, KERNEL_SIZE)

            measured_x = None
            measured_y = None

            contours = get_contours(clean_frame)
            avg_largest_contour_search_time = np.mean(largest_contour_search_times, axis = 0) if i > 0 else 0
            if contours:
                largest_contour_search_time = time.time()
                largest_contour = get_largest_contour(contours)
                measured_x, measured_y = get_contour_origin(largest_contour)
                largest_contour_search_times.append(time.time() - largest_contour_search_time)
            else:
                time.sleep(avg_largest_contour_search_time) #to keep sample rate consistent for KF

            new_x = measured_x
            new_y = measured_y


            if KALMAN_FILTER:
                sane_measurement = is_sane_measurement(
                    kalman_filter, measured_x, measured_y
                ) if i > 100 else True # Assumes the first 100 measurements are accurate to gather values for prediction

                if sane_measurement:
                    if first_measurement:
                        first_measurement = False
                        kalman_filter.errorCovPost = np.eye(4, dtype=np.float32)
                        kalman_filter.statePost = np.array(
                            [[measured_x], [measured_y], [0], [0]], dtype=np.float32)
                    else:
                        kalman_filter.correct(np.array([[measured_x], [measured_y]], dtype=np.float32))
                else:
                    estimated_state = kalman_filter.statePost
                    new_x = estimated_state[0, 0]
                    new_y = estimated_state[1, 0]

                print(f"M {measured_x} {measured_y}\nE {kalman_filter.statePost[0, 0]} {kalman_filter.statePost[1, 0]}\nSane {sane_measurement}")

            delta_x = origin_x - new_x
            delta_y = origin_y - new_y

            results.loc[len(results)] = {"measured_X" : measured_x,
                                         "measured_Y" : measured_y,
                                         "estimated_X" : kalman_filter.statePost[0, 0] if KALMAN_FILTER else new_x,
                                         "estimated_Y" : kalman_filter.statePost[1, 0] if KALMAN_FILTER else new_y,
                                         "delta_x": delta_x,
                                         "delta_y": delta_y,
                                         "Time" : time.time() - start_time}

            if (i + 1) % 100 == 0:
                new_dropped_frames = camera.get_dropped_frames()
                print(f"Iteration: {i + 1}, Update Rate: {camera_stream.get_update_rate()}, Sample Rate: {(i + 1) / (time.time() - start_time)}, Dropped Frames: {new_dropped_frames - old_dropped_frames}")
                old_dropped_frames = new_dropped_frames

            if FSM:
                current_sleep_time = time.time()
                if current_sleep_time - last_sleep_time >= fsm_sleep_time:
                    amplitude_x += delta_x / (distance_x * math.tan(math.radians(50)))
                    amplitude_y -= delta_y / (distance_y * math.tan(math.radians(50)))

                    assert -1.0 <= amplitude_x <= 1.0 and -1.0 <= amplitude_y <= 1.0, f"Amplitudes ({amplitude_x}, {amplitude_y}) must be between -1.0 and 1.0"
                    fsm.send_command(f"xy={amplitude_x};{amplitude_y}", print_sent= False, print_received= False)
                    last_sleep_time = current_sleep_time

            if FSM_LINEAR:
                linear_amp_x += linear_amp_incr_x
                linear_amp_y += linear_amp_incr_y
                fsm.send_command(f"xy={linear_amp_x};{linear_amp_y}", False, False)

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