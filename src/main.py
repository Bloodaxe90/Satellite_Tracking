import os
import pandas as pd
import yaml

from src.kalman_filter.kalman_filter_tuner import tuner
from src.lazer_tracking import lazer_tracking
from src.system_setup_and_calibration import system_setup_and_calibration
from src.utils.io import save_results


def main():
    """
    Main for the laser tracking system

    Performs system setup & calibration and then runs either:
    - Tracking mode: log laser spot position and FSM corrections.
    - Tuning mode: sweep Kalman filter parameters and record performance metrics.
    before saving results and performing cleanup
    """

    ROOT_DIR = os.path.dirname(os.getcwd())
    with open(f"{ROOT_DIR}/config.yaml", "r") as f:
        config = yaml.safe_load(f)

    cam_cfg = config["camera"]
    fsm_cfg = config["fsm"]
    kf_cfg = config["kalman_filter"]

    EXPERIMENT_NAME = (
        f"{config['general']['experiment_name_prefix']}_"
        f"G{cam_cfg['gain']}_"
        f"E{cam_cfg['exposure']}_"
        f"B{cam_cfg['bins']}_"
        f"TR{1 if config['run_mode'] == 'tuner' else 0}_"
        f"LF{1 if config['linear_fsm']['enabled'] else 0}_"
        f"KF{1 if kf_cfg['enabled'] else 0}_"
        f"MOU{kf_cfg['model_uncertainty']}_"
        f"MEU{kf_cfg['measurement_uncertainty']}_"
        f"FR{1 if fsm_cfg['use_fsm_roi'] else 0}"
    )

    camera = None
    camera_stream = None
    fsm = None
    results = None

    try:
        # Setup & calibration
        (
            camera,
            camera_stream,
            master_dark,
            origin_pos,
            fsm,
            fsm_sleep_time,
            distances,
            amplitude_bounds,
            kalman_filter,
        ) = system_setup_and_calibration(
            config=config,
            root_dir=ROOT_DIR,
            amplitude_bounds=(
                fsm_cfg["absolute_min_amplitude"],
                fsm_cfg["absolute_max_amplitude"],
                fsm_cfg["absolute_min_amplitude"],
                fsm_cfg["absolute_max_amplitude"],
            ),
        )

        # Run Mode Selection
        run_mode = config["run_mode"]

        if run_mode == "tracking":
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

            lazer_tracking(
                config=config,
                camera_stream=camera_stream,
                fsm=fsm,
                master_dark=master_dark,
                results=results,
                origin_pos=origin_pos,
                distances=distances,
                fsm_sleep_time=fsm_sleep_time,
                amplitude_bounds=amplitude_bounds,
                kalman_filter=kalman_filter,
            )

        elif run_mode == "tuning":
            results = pd.DataFrame(
                columns=[
                    "model_uncertainty",
                    "measurement_uncertainty",
                    "error_x",
                    "error_y",
                    "sane_rate",
                ]
            )

            tuner(
                config=config,
                camera_stream=camera_stream,
                fsm=fsm,
                results=results,
                master_dark=master_dark,
                origin_pos=origin_pos,
                distances=distances,
                fsm_sleep_time=fsm_sleep_time,
                amplitude_bounds=amplitude_bounds,
            )

        else:
            print(f"Run mode '{run_mode}' is not a valid run mode")

    except (KeyboardInterrupt, RuntimeError, AssertionError) as e:
        print(f"\nTracking stopped due to Error: {e}")

    finally:
        # Cleanup
        print("Cleaning up")
        if camera_stream is not None:
            camera_stream.stop()
        if camera is not None:
            camera.close()
        if fsm is not None:
            fsm.send_command("xy=0;0")
            fsm.disconnect()
        if results is not None:
            save_results(ROOT_DIR, results, f"{EXPERIMENT_NAME}")
        print("Cleanup complete.")


if __name__ == "__main__":
    main()
