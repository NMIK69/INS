import numpy as np
import sys
import matplotlib.pyplot as plt
import math

import math
import numpy as np
import copy
from scipy import signal


def get_median_absolute_deviation(data, mid):
    sdata = copy.deepcopy(data)
    sdata.sort()
    median = sdata[mid]

    abs_deviations = []

    for x in sdata:

        abs_deviations.append(abs(x - median))

    abs_deviations.sort()
    return median, abs_deviations[mid]


def hampelfilter(data, win_size, dev_num):
    # reference: https://de.mathworks.com/help/signal/ref/hampel.html

    side_size = win_size / 2
    L = 1.4826

    median_index = win_size
    start = 0
    mid = win_size
    end = (win_size * 2) + 1

    result = copy.deepcopy(data)

    while (end < len(data)):
        median, mad = get_median_absolute_deviation(data[start:end], median_index)

        diff = abs(data[mid] - median)
        cap = dev_num * L * mad

        if (diff > cap):
            print(mid)
            result[mid] = median

        start += 1
        mid += 1
        end += 1

    return result

def hampelfilter_3d(data, win_size, dev_num):
	result = copy.deepcopy(data)
	result = np.array(result).T

	result[0] = hampelfilter(result[0], win_size, dev_num)
	result[1] = hampelfilter(result[1], win_size, dev_num)
	result[2] = hampelfilter(result[2], win_size, dev_num)

	return result.T

def highpass_filter(data, order, cutoff_freq, sample_rate):

		result = copy.deepcopy(data)
		result = np.array(result).T

		nyq = 0.5 * sample_rate
		normal_cutoff = cutoff_freq / nyq
		b_filt, a_filt = signal.butter(order, normal_cutoff, btype='high', analog=False)
		#b_filt, a_filt = signal.butter(1,(2*cutoff_freq)/(sample_rate),analog=False, btype='high')

		result[0] = signal.filtfilt(b_filt,a_filt,result[0])
		result[1] = signal.filtfilt(b_filt,a_filt,result[1])
		result[2] = signal.filtfilt(b_filt,a_filt,result[2])

		return result.T


def lowpass_filter(data, order, cutoff_freq, sample_rate):

		result = copy.deepcopy(data)
		result = np.array(result).T

		nyq = 0.5 * sample_rate
		normal_cutoff = cutoff_freq / nyq
		b_filt, a_filt = signal.butter(order, normal_cutoff, btype='low', analog=False)
		#b_filt, a_filt = signal.butter(order, cutoff_freq, fs=sample_rate, btype='low', analog=False)

		result[0] = signal.filtfilt(b_filt,a_filt,result[0])
		result[1] = signal.filtfilt(b_filt,a_filt,result[1])
		result[2] = signal.filtfilt(b_filt,a_filt,result[2])

		return result.T


def bandpass_filter(data, order, cutoff_freq_low, cutoff_freq_high, sample_rate):

		result = copy.deepcopy(data)
		result = np.array(result).T

		nyq = 0.5 * sample_rate
		low = cutoff_freq_low / nyq
		high = cutoff_freq_high / nyq

		b_filt, a_filt = signal.butter(order, [low, high], btype='band', analog=False)

		result[0] = signal.filtfilt(b_filt, a_filt, result[0])
		result[1] = signal.filtfilt(b_filt, a_filt, result[1])
		result[2] = signal.filtfilt(b_filt, a_filt, result[2])

		return result.T, b_filt, a_filt

np.set_printoptions(suppress=True, linewidth=180)


def read_file(filename):
	fp = open(filename, "r")
	lines = fp.readlines()
	
	
	accel = []
	gyro = []
	k = 0

	while(k < len(lines)):
		data = lines[k].split(",")

		ax = float(data[1])
		ay = float(data[2])
		az = float(data[3])

		gx = float(data[4])
		gy = float(data[5])
		gz = float(data[6])

		accel.append([ax, ay, az])
		gyro.append([gx, gy, gz])
		
		k += 1
	
	return np.array(accel), np.array(gyro)

filename = sys.argv[1]

accel, gyro = read_file(filename)

#accel = lowpass_filter(accel, 3, 5, 250);
#accel = hampelfilter_3d(accel, 5, 4);
#gyro = hampelfilter_3d(gyro, 5, 4);

time = np.linspace(0, (len(accel) * 1/250.0), num=len(accel))

plt.plot(accel[:, 0], c="r", label="X")
plt.plot(accel[:, 1], c="g", label="Y")
plt.plot(accel[:, 2], c="b", label="Z")
plt.legend()
plt.grid()
plt.show()

#plt.plot(gyro[:, 0], c="r", label="X")
#plt.plot(gyro[:, 1], c="g", label="Y")
#plt.plot(gyro[:, 2], c="b", label="Z")
#plt.legend()
#plt.grid()
#plt.show()

