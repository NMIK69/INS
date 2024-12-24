import numpy as np
import sys
import matplotlib.pyplot as plt
import math
from scipy.optimize import least_squares

np.set_printoptions(suppress=True, linewidth=180)


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


def pretty_print(z):
	print(f'M = np.array([[{z[0]}, {z[1]}, {z[2]}],\n'
	      f'              [{z[3]}, {z[4]}, {z[5]}],\n'
	      f'              [{z[6]}, {z[7]}, {z[8]}]]')
			   
	print()
	print(f'b = np.array([{z[9]}, {z[10]}, {z[11]}])')

def accel_cal_model(z, a_raw):
	M = np.array([[z[0], z[1], z[2]],
		      [z[3], z[4], z[5]],
		      [z[6], z[7], z[8]]])

	b = np.array([z[9], z[10], z[11]])


	cal = M @ (a_raw + b)

	r = (1.0 - (np.linalg.norm(cal)**2))**2
	return r

def residuals(z, a_raw):
	L = 0.0

	for a in a_raw:
		L += accel_cal_model(z, a)

	return L

filename = sys.argv[1]

data = read_accel_from_file(filename)

z0 = np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

res = least_squares(residuals, z0, args=(data, ))
zr = res.x

pretty_print(zr)

cal = []
M = np.array([[zr[0], zr[1], zr[2]],
	      [zr[3], zr[4], zr[5]],
	      [zr[6], zr[7], zr[8]]])
b = np.array([zr[9], zr[10], zr[11]])


for a in data:
	c = M @ (a + b) 
	cal.append([c[0], c[1], c[2]])

cal = np.array(cal)


fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(data[:, 0], data[:, 1], data[:, 2], c="g", label="raw")
ax.scatter(cal[:, 0], cal[:, 1], cal[:, 2], c="r", label="cal")
ax.set_box_aspect([1,1,1])
plt.legend()
plt.show()

plt.scatter(cal[:, 0], cal[:, 1], c="r", label="cal")
plt.scatter(data[:, 0], data[:, 1], c="g", label="raw")
circ = plt.Circle((0, 0), 1, color='b', fill=False, label="expected")
plt.gca().add_patch(circ)
plt.gca().set_aspect('equal')
plt.legend()
plt.grid()
plt.show()

plt.scatter(cal[:, 1], cal[:, 2], c="r", label="cal")
plt.scatter(data[:, 1], data[:, 2], c="g", label="raw")
circ = plt.Circle((0, 0), 1, color='b', fill=False, label="expected")
plt.gca().add_patch(circ)
plt.gca().set_aspect('equal')
plt.legend()
plt.grid()
plt.show()

plt.scatter(cal[:, 0], cal[:, 2], c="r", label="cal")
plt.scatter(data[:, 0], data[:, 2], c="g", label="raw")
circ = plt.Circle((0, 0), 1, color='b', fill=False, label="expected")
plt.gca().add_patch(circ)
plt.gca().set_aspect('equal')
plt.legend()
plt.grid()
plt.show()

