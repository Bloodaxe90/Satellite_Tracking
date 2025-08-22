from typing import Sequence
import cv2
import numpy as np
from cv2 import Mat


def get_clean_frame(
    light_frame: np.ndarray, master_dark: np.ndarray, kernel_size: int = 3
) -> np.ndarray:
    """
    Removes noise from raw light frame by subtracting dark frame and other methods

    Parameters:
        light_frame (np.ndarray): The image captured with light
        master_dark (np.ndarray): The master dark frame
        kernel_size (int): The size of the kernel for morphological opening (Defaults to 3)

    Returns:
        np.ndarray: The cleaned frame
    """
    clean_frame = cv2.subtract(light_frame, master_dark)

    # Use morphological opening to remove small noise blobs in in the background
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    clean_frame = cv2.morphologyEx(clean_frame, cv2.MORPH_OPEN, kernel, iterations=1)
    return clean_frame


def get_contours(image: np.ndarray) -> Sequence[Mat | np.ndarray]:
    """
    Finds and returns the contours in a single channel image

    Parameters:
        image (np.ndarray): A grayscale image (single channel)

    Returns:
        Sequence[Mat | np.ndarray]: A list of contours found in the image
    """
    # Convert the image to a binary mask using Otsus thresholding
    _, mask = cv2.threshold(
        src=image,
        thresh=69,  # This value is ignored because Otsus method is used
        maxval=255,
        type=cv2.THRESH_BINARY + cv2.THRESH_OTSU,
    )
    # Find external contours in the binary mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contours


def get_contour_origin(contour: Mat | np.ndarray) -> tuple[float, float]:
    """
    Finds the origin of a contour using image moments

    Parameters:
        contour (Mat | np.ndarray): A single contour

    Returns:
        tuple[float, float]: The (x, y) coordinates of the contour's origin
    """
    # Calculate image moments (used to find shape properties like area and centre)
    M = cv2.moments(contour)
    largest_contour_area = M["m00"]

    # Make sure the area is not zero to avoid division errors
    assert largest_contour_area > 0, "Cannot find any contours"

    x = M["m10"] // largest_contour_area  # M["m10"] is sum of x coordinates
    y = M["m01"] // largest_contour_area  # M["m01"] is sum of x coordinates

    return x, y


def get_largest_contour(contour: Sequence[Mat | np.ndarray]) -> Mat | np.ndarray:
    """
    Returns the contour with the largest area

    Parameters:
        contour (Sequence[Mat | np.ndarray]): A list of contours

    Returns:
        Mat | np.ndarray: The largest contour found
    """
    return max(contour, key=cv2.contourArea)


def get_stacked_frame(frames: list) -> np.ndarray:
    """
    Compute a single stacked frame from a list of frames by taking
    the pixel median across all frames

    Args:
        frames (list): List of image frames (each as a numpy array)

    Returns:
        np.ndarray: Median stacked image frame as an 8-bit unsigned integer
    """

    stacked_frame = np.median(frames, axis=0).astype(np.uint8)

    return stacked_frame


def get_rotated_frame(frame: np.ndarray, origin_pos: tuple, angle: float) -> np.ndarray:
    """
    Rotates a frame around a specified origin by a given angle.

    Args:
        frame (np.ndarray): Input image to rotate
        origin_pos (tuple): (x, y) coordinates of the rotation origin
        angle (float): Rotation angle in degrees

    Returns:
        np.ndarray: Rotated image/frame
    """

    # Compute 2D rotation matrix for the given origin and angle
    rotation_matrix = cv2.getRotationMatrix2D(origin_pos, angle, 1)

    # Apply rotation to the frame
    rotated_frame = cv2.warpAffine(frame, rotation_matrix, frame.shape)

    return rotated_frame
