import os
import time

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image


def wait(wait_message: str):
    while True:
        waiting = input(f"\n{wait_message} \nY/n ")

        if waiting.lower() == "y":
            break
        time.sleep(0.1)


def show_images(*args, title: str = ""):
    """
    Display one or more images

    Args:
        *args: One or more images to display. Each image can be either:
            - A numpy array
            - A filename (string) of an image located in 'resources/images'
        title (str): Optional title to display above all images.

    Behavior:
        - If no images are provided, prints a message and returns.
        - Loads images from disk if filenames are provided.
        - Supports grayscale (2D arrays) and color images.
    """

    num_images = len(args)  # Number of images to display
    image_dir = os.path.join(os.path.dirname(os.getcwd()), "resources", "images")

    assert num_images > 0, "No images provided"

    # Create subplots in one row with a column per image
    fig, axes = plt.subplots(1, num_images, figsize=(5 * num_images, 5))

    if title != "":
        fig.suptitle(title, fontsize=16)

    # If only one image, axes is not a list by default, so make it a list
    if num_images == 1:
        axes = [axes]

    for i, img in enumerate(args):
        # If the argument is a filename, load the image from the resources directory
        if isinstance(img, str):
            img = np.array(Image.open(f"{image_dir}/{img}"))

        # Display grayscale images with a gray colormap and set value range 0-255
        if img.ndim == 2:
            axes[i].imshow(img, cmap="gray", vmin=0, vmax=255)
        else:
            axes[i].imshow(img)  # Display color images as is

        axes[i].set_title(f"Image {i + 1}")
        axes[i].axis("off")

    plt.tight_layout()
    plt.show()
