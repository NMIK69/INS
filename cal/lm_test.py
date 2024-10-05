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



# working the example from
# https://www.igpm.rwth-aachen.de/Numa/NumaMB/SS10/LevMarq.pdf

data = np.array([[2, 0], [3, 2], [4, 0]])



def make_F(a, b):
	F = []

	for d in data:
		x = d[0]
		y = d[1]

		f = (x - a)**2 + math.pow(math.e, b*(x**2 + y**2)) - 5
		F.append(f)
	
	return np.array(F)


def make_dF(a, b):
	dF = []

	for d in data:
		x = d[0]
		y = d[1]

		dfa = -2 * (x - a)
		dfb = (x**2 + y**2) * math.pow(math.e, b*(x**2 + y**2))

		dF.append([dfa, dfb])

	return np.array(dF)



def lm_step(z, mu):
	I = np.eye(2)
	F = make_F(z[0], z[1])
	dF = make_dF(z[0], z[1])

	cond = (dF.T @ F)
	for c in cond:
		if(math.isnan(c) or abs(c) < 1e-10):
			exit()
	
	s = -np.linalg.inv((dF.T @ dF + mu*I)) @ dF.T @ F
	
	z1 = z + s
	F1 = make_F(z1[0], z1[1])
	FdFs = F + dF @ s 
	
	snF = (np.linalg.norm(F))**2
	snF1 = (np.linalg.norm(F1))**2
	snFdFs = (np.linalg.norm(FdFs))**2
	
	p = (snF - snF1) / (snF - snFdFs)
	
	return s, p




def lm_advance(z, mu, b0, b1):
	s, p = lm_step(z, mu)
	
	#i = 0
	while(p <= b0):
		#tmp = z + s
		#print(f'{i}| p:{p} | mu: {mu} x:{tmp}')
		#i += 1

		mu = mu * 2
		s, p = lm_step(z, mu)


	# now it is certain that either b0 < p < b1 or p >= b1
	if(p >= b1):
		mu = mu / 2.0
	else:
		if(not(b0 < p < b1)):
			print(p)
			assert(b0 < p < b1)
	
	return s, p, mu


def lm_fit(z, mu, b0, b1):
	

	zp = z
	i = 0
	while(1):
		s, p, mu = lm_advance(z, mu, b0, b1)
		z += s

		print(f'{i}| p:{p} | mu:{mu} | x1:{z}')
		i += 1

		if i > 100:
			break
	
	return z


#b0 = 0.2
#b1 = 0.8
#
## initial param estimation
#mu = 1
#z = [4, 0]


z0 = np.array([4.0,0.0])
mu0 = 1.0 
zr = lm_fit(z0, mu0, 0.2, 0.8)

print(zr)

#s, p = lm_step(z0, mu0)
#print(f'p:{p} | mu0:{mu0} | x:{z0 + s}')


#z = np.array([4, 0])
#F = make_F(z[0], z[1])
#dF = make_dF(z[0], z[1])
#
##F = np.vstack((F, np.zeros((2,1))))
##F = np.hstack((F, np.zeros(2)))
##dF = np.vstack((dF, mu*I))
#
#print(F, "\n")
#print(dF, "\n")
#
#
#
#s = -np.linalg.inv((dF.T @ dF + mu*I)) @ dF.T @ F
#
#print(s)
#
#z1 = z + s
#F1 = make_F(z1[0], z1[1])
#FdFs = F + dF @ s 
#
#snF = (np.linalg.norm(F))**2
#snF1 = (np.linalg.norm(F1))**2
#snFdFs = (np.linalg.norm(FdFs))**2
#
#p = (snF - snF1) / (snF - snFdFs)
#
#print(p)


	






