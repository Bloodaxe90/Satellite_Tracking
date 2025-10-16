import math
import os

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


def tuner():
    # Tuner assumes there will always be a measurement and that all measurements are valid and can be given to the Kalman filter

    # General Parameters
    ROOT_DIR = os.path.dirname(os.getcwd())
    KERNEL_SIZE = 3

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

    # Kalman Filter Tuner
    SEARCH_ITERATIONS = 3
    INITIAL_MODEL_UNCERTAINTIES = [1e-6]
    INITIAL_JERK_MODEL_UNCERTAINTIES = [1e-9]
    INITIAL_MEASUREMENT_UNCERTAINTIES = [30]
    NUM_MODEL_UNCERTAINIES = 3

    NUM_JERK_MODEL_UNCERTAINIES = 1

    NUM_MEASUREMENT_UNCERTAINIES = 3
    MODEL_REDUCTION_FACTOR = 0.5
    JERK_MODEL_REDUCTION_FACTOR = 0.5
    MEASUREMENT_REDUCTION_FACTOR = 0.5
    ERROR_WEIGHT = 1
    SANE_WEIGHT = 0

    # testing specific parameters (Including frequency plotter)
    ITERATIONS = 1000
    TIME = 60

    FSM = False

    # Linear FSM
    FSM_LINEAR = True
    initial_amplitude_x = -0.04
    initial_amplitude_y = -0.04
    linear_amp_incr_x = 0.00001
    linear_amp_incr_y = 0.00001
    amp_x_acc = 1.005
    amp_y_acc = 1.005
    amp_x_dec = 2 - amp_x_acc
    amp_y_dec = 2 - amp_y_acc


    EXPERIMENT_NAME: str = (
        f"ca_tuner_"
        f".04A_"
        f"SI{SEARCH_ITERATIONS}_"
        f"I{ITERATIONS}_"
        f"G{GAIN}_"
        f"E{EXPOSURE}_"
        f"B{BINS}_"
        f"R{RESOLUTION}_"
        f"S{START_POS}_"
        f"FSM{1 if FSM else 0}_"
        f"K{KERNEL_SIZE}"
    )

    final_results = pd.DataFrame(
        columns=[
            "model_uncertainty",
            "jerk_model_uncertainty",
            "measurement_uncertainty",
            "error_x",
            "error_y",
            "sane_rate",
        ]
    )

    amplitude_bounds = (MIN_AMPLITUDE, MAX_AMPLITUDE, MIN_AMPLITUDE, MAX_AMPLITUDE)

    camera = None
    camera_stream = None
    fsm = None
    distance_x = None
    distance_y = None

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

        wait("Generate origin?")
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

            print(f"Distance X {distance_x}, Y {distance_y}")

        camera_stream = CameraStream(camera).start()
        if FSM:
            fsm_sleep_time = get_response_time(
                camera_stream, master_dark, fsm, amplitude_bounds
            )
        wait("Start?")

        model_uncertainties = INITIAL_MODEL_UNCERTAINTIES
        measurement_uncertainties = INITIAL_MEASUREMENT_UNCERTAINTIES
        jerk_model_uncertainties = INITIAL_JERK_MODEL_UNCERTAINTIES
        for i in range(SEARCH_ITERATIONS):

            print(
                f"{i + 1}. MOU {model_uncertainties}, JMU {jerk_model_uncertainties}, MEU {measurement_uncertainties}"
            )
            for model_uncertainty in model_uncertainties:
                for jerk_model_uncertainty in jerk_model_uncertainties:
                    for measurement_uncertainty in measurement_uncertainties:

                        print(
                            f"Tested parameters: MOU {model_uncertainty}, JMU {jerk_model_uncertainty}, MEU {measurement_uncertainty}"
                        )
                        error_x = []
                        error_y = []
                        sane_measurements = []

                        if FSM_LINEAR:
                            fsm.send_command(f"xy={initial_amplitude_x};{initial_amplitude_y}", False, False)
                            time.sleep(1)

                        initial_sample_time = 1 / camera_stream.get_update_rate()
                        kalman_filter = setup_kalman_filter(
                            delta_t=initial_sample_time,
                            model_uncertainty=model_uncertainty,
                            jerk_model_uncertainty= jerk_model_uncertainty,
                            measurement_uncertainty=measurement_uncertainty,
                        )
                        start_time = time.time()
                        last_time = start_time
                        first_measurement = True
                        last_sleep_time = start_time
                        amplitude_x = initial_amplitude_x
                        amplitude_y = initial_amplitude_y
                        changed = False
                        j = 0
                        while time.time() - start_time < TIME:

                            current_time = time.time()
                            sample_time = current_time - last_time
                            last_time = current_time
                            set_transition_matrix(kalman_filter, sample_time)

                            kalman_filter.predict()

                            if (amplitude_x > 0 or amplitude_y > 0) and not changed:
                                changed = True
                                current_state = kalman_filter.statePost
                                current_state[4, 0] *= -1  # ax_new = -ax_old
                                current_state[5, 0] *= -1  # ay_new = -ay_old
                                current_state[6, 0] *= -1  # jx_new = -jx_old
                                current_state[7, 0] *= -1  # jy_new = -jy_old

                                kalman_filter.statePost = current_state

                                kalman_filter.statePre = current_state

                            raw_frame = camera_stream.read()
                            if raw_frame is None:
                                time.sleep(0.001)
                                continue

                            clean_frame = get_clean_frame(
                                raw_frame, master_dark, KERNEL_SIZE
                            )

                            contours = get_contours(clean_frame)
                            if not contours:
                                break;

                            largest_contour = get_largest_contour(contours)
                            measured_x, measured_y = get_contour_origin(largest_contour)

                            sane_measurement = is_sane_measurement(
                                kalman_filter, measured_x, measured_y
                            )

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
                                # TODO remove later
                                if TIME * 0.25 < time.time() - start_time < TIME * 0.75:
                                    pass
                                else:
                                    kalman_filter.correct(
                                        np.array([[measured_x], [measured_y]],
                                                 dtype=np.float32)
                                    )

                            estimated_state = kalman_filter.statePost
                            estimated_x = estimated_state[0, 0]
                            estimated_y = estimated_state[1, 0]

                            # print(f"M {measured_x} {measured_y}, E {estimated_x} {estimated_y}, Sane {sane_measurement}")

                            delta_x = origin_x - measured_x
                            delta_y = origin_y - measured_y

                            error_x.append(abs(measured_x - estimated_x))
                            error_y.append(abs(measured_y - estimated_y))
                            sane_measurements.append(sane_measurement)

                            # if (i + 1) % 100 == 0:
                            #     print(f"Iteration: {i + 1}, FPS: {camera_stream.get_fps()}, Sample Rate: {(i + 1) / (time.time() -  start_time)}")

                            if FSM:
                                current_sleep_time = time.time()
                                if current_sleep_time - last_sleep_time >= fsm_sleep_time:
                                    amplitude_x += delta_x / (
                                        distance_x * math.tan(math.radians(50))
                                    )
                                    amplitude_y -= delta_y / (
                                        distance_y * math.tan(math.radians(50))
                                    )

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
                                linear_amp_incr_x *= (
                                    amp_x_acc if amplitude_x < 0 and amplitude_y < 0 else amp_x_dec)
                                linear_amp_incr_y *= (
                                    amp_y_acc if amplitude_x < 0 and amplitude_y < 0 else amp_y_dec)

                                fsm.send_command(
                                    f"xy={amplitude_x};{amplitude_y}", False, False
                                )

                            j += 1

                        print(len(final_results))
                        final_results.loc[len(final_results)] = {
                            "model_uncertainty": model_uncertainty,
                            "measurement_uncertainty": measurement_uncertainty,
                            "jerk_model_uncertainty": jerk_model_uncertainty,
                            "error_x": np.mean(error_x, axis=0),
                            "error_y": np.mean(error_y, axis=0),
                            "sane_rate": np.sum(sane_measurements) / j,
                        }
                        print(final_results)

            rmse = np.sqrt(
                final_results["error_x"] ** 2 + final_results["error_y"] ** 2
            )
            normalized_rmse = (rmse - rmse.min()) / (rmse.max() - rmse.min() + 1e-12)
            cost = (ERROR_WEIGHT * normalized_rmse) + (
                SANE_WEIGHT * (1 - final_results["sane_rate"])
            )
            best_index = np.argmin(cost, axis=0)

            center_value = final_results["model_uncertainty"][best_index]
            half_range = center_value * MODEL_REDUCTION_FACTOR
            new_space = np.linspace(
                center_value - half_range,
                center_value + half_range,
                NUM_MODEL_UNCERTAINIES,
            )
            model_uncertainties = np.array([max(x, 1e-9) for x in new_space]).astype(
                np.float32
            )

            center_value = final_results["jerk_model_uncertainty"][best_index]
            half_range = center_value * JERK_MODEL_REDUCTION_FACTOR
            new_space = np.linspace(
                center_value - half_range,
                center_value + half_range,
                NUM_JERK_MODEL_UNCERTAINIES,
            )

            jerk_model_uncertainties = np.array(
                [max(x, 1e-9) for x in new_space]
            ).astype(np.float32)

            center_value = final_results["measurement_uncertainty"][best_index]
            half_range = center_value * MEASUREMENT_REDUCTION_FACTOR
            new_space = np.linspace(
                center_value - half_range,
                center_value + half_range,
                NUM_MEASUREMENT_UNCERTAINIES,
            )

            measurement_uncertainties = np.array(
                [max(x, 1e-9) for x in new_space]
            ).astype(np.float32)

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
