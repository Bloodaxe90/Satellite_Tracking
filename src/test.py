import math
import os
from collections import deque

import cv2
import numpy as np
import pandas as pd
import time

from src.camera.camera_setup import setup_camera
from src.camera.camera_stream import CameraStream
from src.camera.image_processing import get_clean_frame, \
    get_redness_frame, get_contours, get_contour_origin, get_largest_contour, \
    get_rotated_frame
from src.fsm.fsm_calibration import get_distance_to_camera, get_damping_factor, \
    get_alignment_offset_angle
from src.fsm.fsm_setup import setup_fsm
from src.kalman_filter.kalman_filter_calibration import is_sane_measurement, \
    set_transition_matrix
from src.utils.live_plotter import LivePlotter
from src.utils.general import wait
from src.utils.io import save_results
from src.kalman_filter.kalman_filter_setup import setup_kalman_filter
from src.utils.spectral_analysis import get_frequencies_and_amplitudes


def test():
    # General Parameters
    ROOT_DIR = os.path.dirname(os.getcwd())
    COLOUR = False
    BUFFER_CAPACITY = 1
    KERNEL_SIZE = 3

    # Camera Parameters
    SDK_LIB_NAME: str = 'libASICamera2.dylib'
    CAMERA_ID: int = 0
    BINS = 1
    GAIN = 120
    EXPOSURE = 32
    RESOLUTION = (8288, 5640)
    START_POS = (0, 0)
    FRAMES_DARK = 10

    # FSM Parameters
    PORT: str = "/dev/cu.usbmodem00000000001A1"
    BAUDRATE: int = 256000
    TIMEOUT: int = 1

    # Kalman Filter Parameters
    #TODO need tuning
    MODEL_UNCERTAINTY = 1
    MEASUREMENT_UNCERTAINTY = 2

    # testing specific parameters (Including frequency plotter)
    ITERATIONS = 1000
    KALMAN_FILTER = True
    FSM = False
    CONTOUR_MODE = True

    # Plotter
    PLOTTER = False
    WINDOW_SIZE = 100
    PLOT_CAPACITY = ITERATIONS
    PLOT_UPDATE_RATE = 1
    PLOTTER_Y_FIELDS = (
        ("r-", "X Dominant Frequency"),
        ("b-", "Y Dominant Frequency")
    )
    PLOT_SAVE_NAME = "freq_plot"
    PLOT_TITLE = "Frequency vs Time"
    PLOT_X_LABEL = "Time (s)"
    PLOT_Y_LABEL = "Frequency (Hz)"

    # Linear FSM
    FSM_LINEAR  = True
    linear_amp_x = -0.04
    linear_amp_y = -0.04
    linear_amp_incr_x = 0.00001
    linear_amp_incr_y = 0.00001

    EXPERIMENT_NAME: str = (f"final_tracking_"
                            f".003A_"
                            f"KF{1 if KALMAN_FILTER else 0}_"
                            f"G{GAIN}_"
                            f"E{EXPOSURE}_"
                            f"B{BINS}_"
                            f"R{RESOLUTION}_"
                            f"S{START_POS}_"
                            f"BC{BUFFER_CAPACITY}_"
                            f"FSM{1 if FSM else 0}_"
                            f"MOU{MODEL_UNCERTAINTY}_"
                            f"MEU{MEASUREMENT_UNCERTAINTY}_"
                            f"C{1 if CONTOUR_MODE else 0}_"
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
    plotter = None
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
                                           num_frames=FRAMES_DARK,
                                           colour=COLOUR)

        if FSM_LINEAR:
            fsm: FSM = setup_fsm(PORT, BAUDRATE, TIMEOUT)
            fsm.send_command(f"xy={linear_amp_x};{linear_amp_y}", False, False)
            time.sleep(1)

        if FSM:
            fsm: FSM = setup_fsm(PORT, BAUDRATE, TIMEOUT)

        origin_x, origin_y = ((RESOLUTION[0] / BINS) / 2, (RESOLUTION[1] / BINS) / 2)

        #TODO remove later
        origin_frames = [get_clean_frame(camera.capture(), master_dark) for
                            _ in range(10)]
        origin_image = np.median(origin_frames, axis=0).astype(np.uint8)
        contours = get_contours(origin_image)
        if contours:
            largest_contour = get_largest_contour(contours)
            origin_x, origin_y = get_contour_origin(largest_contour)


        if FSM:
            wait("Ready to calibrate distance from camera.\n"
                 "Please ensure the laser points to around the center of the camera")

            distance_x, distance_y = get_distance_to_camera(camera=camera,
                master_dark=master_dark,
                origin_pos=(origin_x, origin_y),
                fsm=fsm)

            print(f"Distance X{distance_x}, Y{distance_y}")

        frame_buffer: deque = deque(maxlen=BUFFER_CAPACITY)

        camera_stream = CameraStream(camera).start()


        if PLOTTER:
            plotter = LivePlotter(ROOT_DIR,
                                  PLOTTER_Y_FIELDS,
                                PLOT_SAVE_NAME,
                                PLOT_CAPACITY,
                                PLOT_X_LABEL,
                                PLOT_Y_LABEL,
                                PLOT_TITLE)

        if KALMAN_FILTER:
            initial_sample_time = 1 / camera_stream.get_fps()
            kalman_filter = setup_kalman_filter(delta_t=initial_sample_time,
                                                model_uncertainty=MODEL_UNCERTAINTY,
                                                measurement_uncertainty=MEASUREMENT_UNCERTAINTY)

        wait("Start?")
        largest_contour_search_times = deque(maxlen=10)
        old_delta_x = 0
        old_delta_y = 0
        start_time = time.time()
        last_time = start_time
        i = 0

        while time.time() - start_time < 120:

            if KALMAN_FILTER:
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
            if COLOUR and camera.get_camera_property()["IsColorCam"]:
                clean_frame = get_redness_frame(clean_frame)

            if BUFFER_CAPACITY > 1:
                frame_buffer.append(clean_frame)

                clean_frame = (np.median(
                    np.stack(frame_buffer, axis=0), axis=0)
                .astype(np.uint8))

            measured_x = None
            measured_y = None

            if CONTOUR_MODE:
                contours = get_contours(clean_frame)
                avg_largest_contour_search_time = np.mean(largest_contour_search_times, axis = 0) if i > 0 else 0

                if contours:
                    largest_contour_search_time = time.time()
                    largest_contour = get_largest_contour(contours)
                    measured_x, measured_y = get_contour_origin(largest_contour)
                    largest_contour_search_times.append(time.time() - largest_contour_search_time)
                else:
                    time.sleep(avg_largest_contour_search_time) #to keep sample rate consistent for KF
            else:
                _, _, _, max_loc = cv2.minMaxLoc(clean_frame)
                measured_x, measured_y = max_loc

            new_x = measured_x
            new_y = measured_y

            if KALMAN_FILTER:
                sane_measurement = is_sane_measurement(
                    kalman_filter, measured_x, measured_y
                ) if i > 100 else True

                if measured_x is not None and measured_x is not None and sane_measurement:
                    if i == 0:
                        kalman_filter.errorCovPost = np.eye(4, dtype=np.float32)
                        kalman_filter.statePost = np.array(
                            [[measured_x], [measured_y], [0], [0]], dtype=np.float32)
                    else:
                        kalman_filter.correct(np.array([[measured_x], [measured_y]], dtype=np.float32))
                estimated_state = kalman_filter.statePost
                new_x = estimated_state[0, 0]
                new_y = estimated_state[1, 0]


                print(f"M {measured_x} {measured_y}\nP {new_x} {new_y}\nSane {sane_measurement}")

            if not KALMAN_FILTER:
                if new_x is not None:
                    delta_x = origin_x - new_x
                else:
                    delta_x = None
                if new_y is not None:
                    delta_y = origin_y - new_y
                else:
                    delta_y = None
            else:
                delta_x = origin_x - new_x
                delta_y = origin_y - new_y

            results.loc[len(results)] = {"measured_X" : measured_x,
                                         "measured_Y" : measured_y,
                                         "estimated_X" : new_x,
                                         "estimated_Y" : new_y,
                                         "delta_x": delta_x,
                                         "delta_y": delta_y,
                                         "Time" : time.time() - start_time}

            if (i + 1) % 100 == 0:
                print(f"Iteration: {i + 1}, FPS: {camera_stream.get_fps()}, Sample Rate: {(i + 1) / (time.time() -  start_time)}")

            if PLOTTER:
                if (i + 1) % PLOT_UPDATE_RATE == 0 and i != 0: # Doesnt wait for buffer to fill before updating to ensure sample rate stays consistent
                    amount = (i + 1) if i < WINDOW_SIZE else WINDOW_SIZE
                    x_positions = np.array(results["X"])[-amount:]
                    y_positions = np.array(results["Y"])[-amount:]
                    times = np.array(results["Time"])[-amount:]
                    x_frequencies, x_amplitudes = get_frequencies_and_amplitudes(x_positions, times)
                    y_frequencies, y_amplitudes = get_frequencies_and_amplitudes(y_positions, times)
                    x_dominant_frequency = x_frequencies[np.argmax(x_amplitudes)]
                    y_dominant_frequency = y_frequencies[np.argmax(y_amplitudes)]
                    plotter.update((x_dominant_frequency, y_dominant_frequency), times[-1])

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

            i += 1

    except (KeyboardInterrupt, RuntimeError, AssertionError) as e:
        print(f"\nTracking stopped due to Error: {e}")

    finally:
        print("Cleaning up")
        if plotter is not None:
            plotter.close()
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