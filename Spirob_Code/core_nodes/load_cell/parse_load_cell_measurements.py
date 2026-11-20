import matplotlib.pyplot as plt
import argparse
import numpy as np
from scipy.signal import butter, filtfilt

measurements = []

parser = argparse.ArgumentParser(description="Parse load cell measurements from a text file")
parser.add_argument('-f', '--file', default='measurements.txt', help="Name of the measurements text file")
args = parser.parse_args()

with open(args.file, 'r') as f:
    # timestamp, elapsed_time = f.readline().split(',')
    # measurements = list(map(int, f.readline().split(',')))
    measurements = list(map(int, f.readline().split(',')))
    elapsed_times = list(map(float, f.readline().split(',')))

print(f"Analyzing measurements for {args.file}")
N = len(measurements)
print(f" - {N} Measurements were taken\n")

class MedianFilter:
    def __init__(self, window_size=5) -> None:
        self.data = []
        self.window_size = window_size
    
    def update(self, raw_value):
        if len(self.data) > self.window_size:
            self.data.pop()
            self.data.insert(0, raw_value)
        else:
            self.data.insert(0, raw_value)
        return np.median(self.data)

class MovingAverageFilter:
    def __init__(self, alpha):
        self.alpha = alpha  # Filter coefficient (0 < alpha < 1)
        self.filtered_value = None

    def update(self, raw_value):
        if self.filtered_value is None:
            self.filtered_value = raw_value
            return self.filtered_value
        self.filtered_value = self.alpha * raw_value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value

def butter_lowpass_filter(data, cutoff, fs, order=5):
    """
    Applies a Butterworth low-pass filter to the input data.

    Args:
        data (np.ndarray): The input signal to be filtered.
        cutoff (float): The cutoff frequency of the filter in Hz.
        fs (float): The sampling frequency of the signal in Hz.
        order (int, optional): The order of the filter. Defaults to 5.

    Returns:
        np.ndarray: The filtered signal.
    """
    nyquist = 0.5 * fs  # Nyquist frequency
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    filtered_data = filtfilt(b, a, data) # Apply the filter forwards and backwards to achieve zero-phase
    return filtered_data

mf = MedianFilter(window_size=10)
maf = MovingAverageFilter(alpha=0.1)

mf_measurements = []
for m in measurements:
    mf_measurements.append(mf.update(m))

maf_measurements = []
for m in measurements:
    maf_measurements.append(maf.update(mf.update(m)))

blpf_measurements = butter_lowpass_filter(mf_measurements, 3, 10)

fig, axs = plt.subplots(2, 2, figsize=(10, 16), constrained_layout=True)

axs[0][0].plot(elapsed_times, measurements)
axs[0][0].set_title("Raw Measurements")
axs[0][0].set_ylabel("Measurement")

axs[0][1].plot(elapsed_times, mf_measurements)
axs[0][1].set_title("Median Filtered Measurements")
axs[0][1].set_ylabel("Measurement")

axs[1][0].plot(elapsed_times, maf_measurements)
axs[1][0].set_title("Moving Average Filtered Measurements")
axs[1][0].set_ylabel("Measurement")

axs[1][1].plot(elapsed_times, blpf_measurements)
axs[1][1].set_title("Butterworth Lowpass Filtered Measurements")
axs[1][1].set_ylabel("Measurement")

plt.tight_layout()
plt.show()