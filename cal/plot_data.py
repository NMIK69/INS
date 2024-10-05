import numpy as np
import sys
import matplotlib.pyplot as plt
import math

np.set_printoptions(suppress=True, linewidth=180)

M = np.array([[1.0040901592060874, 4.222987022681737e-05, 0.0049019930163562864],
              [4.4680702991560335e-05, 0.9696004568787703, 0.009465466115732863],
              [0.004903901923882621, 0.009466845301561033, 1.022226875545407]])

b = np.array([-0.027031603511197856, 0.04020491089323597, -0.046552255173972254])

def read_accel_from_file(filename):
	fp = open(filename, "r")
	lines = fp.readlines()
	
	
	accel = []
	k = 0

	while(k < len(lines)):
		data = lines[k].split(",")

		ax = float(data[0])
		ay = float(data[1])
		az = float(data[2])

		accel.append([ax, ay, az])
		
		k += 1
	
	return np.array(accel)

filename = sys.argv[1]

raw = read_accel_from_file(filename)

cal = []

for a in raw:
	c = M @ (a + b) 
	cal.append([c[0], c[1], c[2]])

cal = np.array(cal)

plt.scatter(cal[:, 0], cal[:, 1], c="r", label="cal")
plt.scatter(raw[:, 0], raw[:, 1], c="g", label="raw")

plt.gca().set_aspect('equal')

plt.legend()
plt.grid()
plt.show()

