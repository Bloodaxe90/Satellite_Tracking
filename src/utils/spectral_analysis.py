import numpy as np


def get_frequencies_and_amplitudes(
    positions: np.ndarray, times: np.ndarray
) -> tuple[np.ndarray, float]:
    """
    Computes the frequencies of the input signal data using a FFT
    (won't lie code was taken from stack overflow as I am not physics student)

    Args:
        positions (np.ndarray): The center of the laser positions (e.g. x, y pixel location)
        times (np.ndarray): Corresponding time values for the data points

    Returns:
        frequencies (np.ndarray): Array of frequencies computed by FFT.
        amplitudes (np.ndarray): Amplitude corresponding to each frequency.
    """
    n = len(positions)

    # Remove DC component by subtracting mean, then compute FFT
    fft_values = np.fft.fft(positions - positions.mean())

    # Compute the normalized amplitudes
    amplitudes = np.abs(fft_values) / n

    # Calculate frequencies corresponding to FFT bins
    freq_resolution = (times[-1] - times[0]) / n
    frequencies = np.fft.fftfreq(n, d=freq_resolution)

    # Keep only positive frequencies
    positive_freq_indices = np.where(frequencies >= 0)
    frequencies = frequencies[positive_freq_indices]
    amplitudes = amplitudes[positive_freq_indices]

    return frequencies, amplitudes


import numpy as np
from scipy.signal import find_peaks, peak_widths


def get_distinct_peak_indices(
    frequencies: np.ndarray, amplitudes: np.ndarray, num_std_devs: int, height: float
) -> list:
    """
    Identifies distinct peaks by filtering insignificant peaks and peaks within the FWHM of another larger peak (if height = 0.5)

    Args:
        frequencies (np.ndarray): Array of frequency values
        amplitudes (np.ndarray): Corresponding amplitude values
        num_std_devs (int): Number of standard deviations above the mean to use as a threshold
        height (float): Relative height used in width calculation (0–1)

    Returns:
        list: List of indices representing distinct, non-overlapping peaks
    """
    # Calculate dynamic threshold based on mean and standard deviation
    mean = np.mean(amplitudes)
    std_dev = np.std(amplitudes)
    threshold = mean + (std_dev * num_std_devs)

    # Find initial candidate peaks above the threshold
    peak_indices: np.ndarray = find_peaks(amplitudes, height=threshold)[0]

    # Calculate the width of each peak (used to check for overlap)
    width_peaks = peak_widths(amplitudes, peak_indices, rel_height=height)[0]

    # Sort peaks by descending amplitude
    sorted_peak_indices: list = sorted(
        peak_indices, key=lambda idx: amplitudes[idx], reverse=True
    )

    distinct_peak_indices = []
    removed_peak_indices = []

    for peak_index in sorted_peak_indices:
        if peak_index in removed_peak_indices:
            continue

        half_peak_width = width_peaks[np.where(peak_indices == peak_index)[0][0]] / 2
        distinct_peak_indices.append(peak_index)

        # Remove nearby peaks considered overlapping (within the FWHM)
        for other_peak_index in sorted_peak_indices:
            if other_peak_index == peak_index:
                continue

            distance = abs(frequencies[other_peak_index] - frequencies[peak_index])
            if distance < half_peak_width:
                removed_peak_indices.append(other_peak_index)

    return distinct_peak_indices
