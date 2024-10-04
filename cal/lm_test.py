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


b0 = 0.2
b1 = 0.8
mu = 1

z = np.array([4, 0])
F = make_F(z[0], z[1])
dF = make_dF(z[0], z[1])
I = np.eye(2)

#F = np.vstack((F, np.zeros((2,1))))
#F = np.hstack((F, np.zeros(2)))
#dF = np.vstack((dF, mu*I))

print(F, "\n")
print(dF, "\n")



s = -np.linalg.inv((dF.T @ dF + mu*I)) @ dF.T @ F

print(s)

z1 = z + s
F1 = make_F(z1[0], z1[1])
FdFs = F + dF @ s 

snF = (np.linalg.norm(F))**2
snF1 = (np.linalg.norm(F1))**2
snFdFs = (np.linalg.norm(FdFs))**2

p = (snF - snF1) / (snF - snFdFs)

print(p)


