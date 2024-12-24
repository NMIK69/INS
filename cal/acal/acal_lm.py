import numpy as np
import sys
import matplotlib.pyplot as plt
import math

np.set_printoptions(suppress=True, linewidth=180)

# used ressources and references:
# https://www.youtube.com/watch?v=n9d3vtWFMRo
# https://www.igpm.rwth-aachen.de/Numa/NumaMB/SS10/LevMarq.pdf
# https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm
# https://kops.uni-konstanz.de/server/api/core/bitstreams/2c6d1313-bb41-4af4-bb20-d4f4c7e32e2b/content

data = []

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

def make_F(z):
	F = []

	m1 = z[0]
	m2 = z[1]
	m3 = z[2]
	m4 = z[3]
	m5 = z[4]
	m6 = z[5]
	m7 = z[6]
	m8 = z[7]
	m9 = z[8]
	bx = z[9]
	by = z[10]
	bz = z[11]


	for d in data:
		ax = d[0]
		ay = d[1]
		az = d[2]

		f1 = ((ax + bx)*m1 + (ay + by)*m2 + (az + bz)*m3)**2
		f2 = ((ax + bx)*m4 + (ay + by)*m5 + (az + bz)*m6)**2
		f3 = ((ax + bx)*m7 + (ay + by)*m8 + (az + bz)*m9)**2
		f = f1 + f2 + f3 - 1.0

		
		F.append(f)
	
	return np.array(F)


def make_dF(z):
	dF = []

	m1 = z[0]
	m2 = z[1]
	m3 = z[2]
	m4 = z[3]
	m5 = z[4]
	m6 = z[5]
	m7 = z[6]
	m8 = z[7]
	m9 = z[8]
	bx = z[9]
	by = z[10]
	bz = z[11]


	for d in data:
		ax = d[0]
		ay = d[1]
		az = d[2]

		dm1 = 2*(ax + bx)*((ax + bx)*m1 + (ay + by)*m2 + (az + bz)*m3)
		dm2 = 2*(ay + by)*((ax + bx)*m1 + (ay + by)*m2 + (az + bz)*m3)
		dm3 = 2*(az + bz)*((az + bz)*m3 + (ax + bx)*m1 + (ay + by)*m2)

		dm4 = 2*(ax + bx)*((ax + bx)*m4 + (ay + by)*m5 + (az + bz)*m6)
		dm5 = 2*(ay + by)*((ax + bx)*m4 + (ay + by)*m5 + (az + bz)*m6)
		dm6 = 2*(az + bz)*((ax + bx)*m4 + (ay + by)*m5 + (az + bz)*m6)

		dm7 = 2*(ax + bx)*((ax + bx)*m7 + (ay + by)*m8 + (az + bz)*m9)
		dm8 = 2*(ay + by)*((ax + bx)*m7 + (ay + by)*m8 + (az + bz)*m9)
		dm9 = 2*(az + bz)*((ax + bx)*m7 + (ay + by)*m8 + (az + bz)*m9)

		dbx =  2*(m1*((az + bz)*m3 + (ax + bx)*m1 + (ay + by)*m2) + m4*((ax + bx)*m4 + (ay + by)*m5 + (az + bz)*m6) + m7*((ax + bx)*m7 + (ay + by)*m8 + (az + bz)*m9))
		
		dby = 2*m2*((az + bz)*m3 + (ax + bx)*m1 + (ay + by)*m2) + 2*m5*((ax + bx)*m4 + (ay + by)*m5 + (az + bz)*m6) + 2*m8*((ax + bx)*m7 + (ay + by)*m8 + (az + bz)*m9)

		dbz = 2*m3*((az + bz)*m3 + (ax + bx)*m1 + (ay + by)*m2) + 2*m6*((ax + bx)*m4 + (ay + by)*m5 + (az + bz)*m6) + 2*m9*((ax + bx)*m7 + (ay + by)*m8 + (az + bz)*m9)

	
		dF.append([dm1, dm2, dm3, dm4, dm5, dm6, dm7, dm8, dm9, dbx, dby, dbz])

	return np.array(dF)


def lm_step(z, mu):
	I = np.eye(12)
	F = make_F(z)
	dF = make_dF(z)

	cond = (dF.T @ F)
	for c in cond:
		if(math.isnan(c) or abs(c) < 1e-10):
			return 1, 0, 0
			#exit()
	
	s = -np.linalg.inv((dF.T @ dF + mu*I)) @ dF.T @ F
	
	z1 = z + s
	F1 = make_F(z1)
	FdFs = F + dF @ s 
	
	snF = (np.linalg.norm(F))**2
	snF1 = (np.linalg.norm(F1))**2
	snFdFs = (np.linalg.norm(FdFs))**2
	
	if((snF - snFdFs) == 0):
		return 1, 0, 0
		#exit()
		
	p = (snF - snF1) / (snF - snFdFs)
	
	return 0, s, p




def lm_advance(z, mu, b0, b1):
	done, s, p = lm_step(z, mu)
	
	#i = 0
	while(p <= b0 and done == 0):
		mu = mu * 2
		done, s, p = lm_step(z, mu)

	if(done != 0):
		return done, s, p, mu

	# now it is certain that either b0 < p < b1 or p >= b1
	if(p >= b1):
		mu = mu / 2.0
	else:
		assert(b0 < p < b1)
	
	return done, s, p, mu


def lm_fit(z, mu, b0, b1):
	done = 0;
	i = 0
	while(done == 0):
		done, s, p, mu = lm_advance(z, mu, b0, b1)
		z += s

		#print(f'{i}| p:{p} | mu:{mu} | x1:{z}')
		i += 1

		if i > 100:
			break
	
	return z

def pretty_print(z):
	print(f'M = np.array([[{z[0]}, {z[1]}, {z[2]}],\n'
	      f'              [{z[3]}, {z[4]}, {z[5]}],\n'
	      f'              [{z[6]}, {z[7]}, {z[8]}]])')
			   
	print()
	print(f'b = np.array([{z[9]}, {z[10]}, {z[11]}])')


filename = sys.argv[1]

data = read_accel_from_file(filename)

z0 = np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
mu0 = 1.0 

zr = lm_fit(z0, mu0, 0.2, 0.8)
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

