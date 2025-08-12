import os

from src.camera.camera_setup import setup_camera
from src.camera.camera_stream import CameraStream
from src.kalman_filter.kalman_filter_setup import setup_kalman_filter
from src.fsm.fsm_calibration import get_amplitude_per_pixel
from src.fsm.fsm import FSM
from src.fsm.fsm_setup import setup_fsm
from src.lazer_tracking import lazer_tracking
from src.utils.general import wait



def main():
    # General Parameters
    ROOT_DIR = os.path.dirname(os.getcwd())   # Root directory of project
    COLOUR = False                            # Whether to use color channel processing
    BUFFER_CAPACITY = 1                       # Number of frames to buffer for median filtering
    KERNEL_SIZE = 3                           # Kernel size for image noise cleaning

    # Camera Parameters
    SDK_LIB_NAME: str = 'libASICamera2.dylib' # SDK library file name for ZWO camera (for macOS, will need to change depending on OS)
    CAMERA_ID: int = 0                        # Index of the camera to use
    BINS = 2                                  # Binning factor to reduce resolution and increase sample rate
    GAIN = 120                                # Camera gain setting
    EXPOSURE = 32                             # Exposure time in microseconds
    RESOLUTION = (8288, 5640)                 # Full sensor resolution
    START_POS = (0, 0)                        # Starting pixel position for ROI
    FRAMES_DARK = 10                          # Number of frames to stack for median dark frame

    # FSM Parameters
    PORT: str = "/dev/cu.usbmodem00000000001A1"  # Serial port for FSM
    BAUDRATE: int = 256000                    # Communication baudrate
    TIMEOUT: int = 1                          # Serial timeout (seconds)

    # Kalman Filter Parameters
    MODEL_UNCERTAINTY = 0.05                  # Process noise covariance
    MEASUREMENT_UNCERTAINTY = 0.5             # Measurement noise covariance

    camera = None
    camera_stream = None
    fsm = None

    try:
        # Initialize camera and generate a master dark frame
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

        # Initialize the FSM (Fast Steering Mirror)
        fsm: FSM = setup_fsm(PORT, BAUDRATE, TIMEOUT)

        wait("Ready to calibrate amplitude per pixel.\n"
             "Please ensure the laser points to around the center of the camera")

        fsm.send_command("control strategy feedforward")

        # Define center pixel coordinates (used as laser target origin)
        origin_pos = ((RESOLUTION[0] / BINS) / 2, (RESOLUTION[1] / BINS) / 2)

        # Determine how much FSM amplitude corresponds to 1 pixel shift
        amplitude_per_pixel = get_amplitude_per_pixel(
            camera=camera,
            master_dark=master_dark,
            origin_pos=origin_pos,
            fsm=fsm)

        print(f"Amplitude Per Pixel X: {amplitude_per_pixel[0]} Y: {amplitude_per_pixel[1]}")

        # Start the threaded camera stream
        camera_stream = CameraStream(camera).start()

        # Create a Kalman filter
        kalman_filter = setup_kalman_filter(frame_rate=camera_stream.get_fps(),
                                            model_uncertainty=MODEL_UNCERTAINTY,
                                            measurement_uncertainty=MEASUREMENT_UNCERTAINTY)

        # Begin real-time laser tracking loop
        lazer_tracking(camera,
                       master_dark,
                       camera_stream,
                       origin_pos,
                       amplitude_per_pixel,
                       fsm,
                       kalman_filter,
                       BUFFER_CAPACITY,
                       KERNEL_SIZE,
                       COLOUR)

    except (KeyboardInterrupt, RuntimeError, AssertionError) as e:
        print(f"\nTracking stopped due to Error: {e}")

    finally:
        # Ensure hardware and threads are cleaned up properly
        print("Cleaning up")
        if fsm is not None:
            fsm.send_command("control strategy off")
            fsm.disconnect()
        if camera_stream is not None:
            camera_stream.stop()
        if camera is not None:
            camera.close()
        print("Cleanup complete.")


if __name__ == "__main__":
    main()
