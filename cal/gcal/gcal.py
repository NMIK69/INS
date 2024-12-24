import numpy as np
import sys
import matplotlib.pyplot as plt
import math
from scipy.optimize import least_squares

np.set_printoptions(suppress=True, linewidth=180)


def read_data(filename):
	fp = open(filename, "r")
	lines = fp.readlines()

	ua = []
	u0 = []
	g = []

	tmp = []
	for line in lines:
		data = line.split(",")

		if(line[0] == "X"):
			g.append(tmp[1:-1])
			ua.append(tmp[-1])
			u0.append(tmp[0])
			tmp.clear()
		else:
			tmp.append([float(data[0]), float(data[1]), float(data[2])])
	
	return ua, u0, g
	
filename = sys.argv[1]

ua, u0, g = read_data(filename)


beta = np.array([1,0,0,0,1,0,0,0,1])


def quat_mul(q, p):
    p0, p1, p2, p3 = p
    q0, q1, q2, q3 = q

    w = p0*q0 - q1*p1 - q2*p2 - q3*p3
    x = q1*p0 + q0*p1 + q2*p3 - q3*p2
    y = q2*p0 + q0*p2 + q3*p1 - q1*p3
    z = q3*p0 + q0*p3 + q1*p2 - q2*p1

    return np.array([w,x,y,z])

def norm_quat(q):
    a = q[0]
    b = q[1]
    c = q[2]
    d = q[3]

    n = np.sqrt(a**2 + b**2 + c** 2 + d**2)

    return [a / n, b / n, c / n, d / n]

def quat_conjugate(q):
    w = q[0]
    x = q[1] * - 1.0
    y = q[2] * - 1.0
    z = q[3] * - 1.0

    return [w, x, y, z]


def quat_inverse(q):
    quat = quat_conjugate(q)
    quat = norm_quat(quat)

    return quat

def quat_integration(gyro, samplerate):
	dt = 1/samplerate
	qn1 = np.array([1,0,0,0])
	q = []
	for g in gyro:
		qg = [0, g[0], g[1], g[2]]
		qn = np.array(qn1) + dt * (0.5 * np.array(quat_mul(qn1, qg)))
		q.append(qn)
		qn1 = qn

	return q

def get_residual(b, u0, ua, gi):
	gc = []

	M = np.array(([b[0], b[1], b[2]],
	             [b[3], b[4], b[5]],
		     [b[6], b[7], b[8]]))
	
	for i in range(len(gi)):
		gc.append(M@(-gi[i]))
	
	
	qg = quat_integration(gc, 250)[-1]
	qu0 = [0.0, u0[0], u0[1], u0[2]]
	
	tmp = quat_mul(quat_mul(qg, qu0), quat_inverse(qg))		
	tmp = norm_quat(tmp)
	ug = np.array([tmp[1], tmp[2], tmp[3]])

	r = ua - ug
	return r, ug
	

def model_1(b, ua, u0, g):
	L = np.array([0.0,0.0,0.0])	
	rs = []
	ugs = []
	
	for i in range(1, len(ua)):
		
		r, ug = get_residual(b, np.array(u0[i]), np.array(ua[i]), np.array(g[i]))
		rs.append(r)
		ugs.append(ug)
		L += np.array(r)

	return L, rs, ugs

def model_2(b, ua, u0, g):
	L = 0.0 
	rs = []
	ugs = []
	
	for i in range(1, len(ua)):
		
		r, ug = get_residual(b, np.array(u0[i]), np.array(ua[i]), np.array(g[i]))
		rs.append(r)
		ugs.append(ug)
		L +=  ((np.linalg.norm(r))**2)

	return L, rs, ugs

def ls_res(bi, ai, ui, gi):
	L, r, u = model_2(bi, ai, ui, gi)
	return L

res = least_squares(ls_res, beta, args=(ua, u0, g, ))

def pretty_print(z):
	print(f'M = np.array([[{z[0]}, {z[1]}, {z[2]}],\n'
	      f'              [{z[3]}, {z[4]}, {z[5]}],\n'
	      f'              [{z[6]}, {z[7]}, {z[8]}]])')
			   
pretty_print(res.x)
