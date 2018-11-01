#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import math
import numpy as np

import math
cimport cython
cimport numpy as np
from cpython cimport array
import array

cdef extern from "math.h" :
  double sin(double x)
  double cos(double x)
  double fabs(double x)

# helper function to create matrix of size x size
cdef np.ndarray[np.float_t, ndim=2] ccreatemat(double[:] ptr, int size) :
  return np.matrix(ptr).reshape(3,3)

# helper function for nxn matrix multiplication R = A*B
cdef cmatmul(double[:,:] A,  double[:,:] B, double[:,:] R, int n) :
  cdef int idx
  cdef double s
  for i in range(n) :
    for j in range(n) :
      s = 0.0
      for k in range(n) :
        s = s + A[i,k] * B[k,j]
      R[i,j] = s

# create 3x3 rotation matrix
cdef np.ndarray[np.float_t, ndim=2] c_rotmat(double angle) :
  cdef double c = cos(angle)
  cdef double s = sin(angle)
  cdef double[9] rot_raw = [c, -s, 0, s, c, 0, 0, 0, 1]
  return np.matrix(rot_raw).reshape(3,3)

@cython.boundscheck(False) 
@cython.wraparound(False)
cpdef get_ref_state(double time, np.ndarray[np.float_t, ndim=1] x, np.ndarray[np.float_t, ndim=1] y, np.ndarray[np.float_t, ndim=1] w, np.ndarray[np.float_t, ndim=1] u, np.ndarray[np.float_t, ndim=1] v, np.ndarray[np.float_t, ndim=1] r, np.ndarray[np.float_t, ndim=1] t, int n) :
  cdef double[3] p_ret
  cdef double[3] v_ret
  cdef (double, double) rx, ry, rw, rvx, rvy, rvw
  cdef double dt
  cdef int i
  cdef int l = n
  rx, ry, rw = (x[0], x[0]), (y[0], y[0]), (w[0], w[0])
  rvx, rvy, rvw = (u[0], u[0]), (v[0], v[0]), (r[0], r[0])
  if time < t[0] : 
    dt = time - t[0]
  elif time > t[n-1] :
    rx, ry, rw = (x[n-1], x[n-1]), (y[n-1], y[n-1]), (w[n-1], w[n-1])
    rvx, rvy, rvw = (u[n-1], u[n-1]), (v[n-1], v[n-1]), (r[n-1], r[n-1])
    dt = time - t[n-1]
  else :
    for i in range(n) :
      if i == 0 : continue
      elif ((time >= t[i-1]) and (time <= t[i])) :
        rx, ry, rw = (x[i-1], x[i]), (y[i-1], y[i]), (w[i-1], w[i])
        rvx, rvy, rvw = (u[i-1], u[i]), (v[i-1], v[i]), (r[i-1], r[i])
        dt = time - t[i-1]
        break
  cdef np.ndarray[np.float_t, ndim=2] pvec
  cdef np.ndarray[np.float_t, ndim=2] vvec
  cdef double dw = (rw[1]-rw[0])
  if fabs(dw) > math.pi :
    dw = (-2*math.pi + dw) if dw > 0 else (2*math.pi + dw)
  cdef double theta = rw[0] + dw * dt
  if fabs(theta) > math.pi :
    theta = (-2*math.pi + theta) if theta > 0 else (2*math.pi + theta)
  pvec = np.matrix([(rx[0] + (rx[1]-rx[0]) * dt), (ry[0] + (ry[1]-ry[0]) * dt), theta]).transpose()
  vvec = np.matrix([(rvx[0] + (rvx[1]-rvx[0]) * dt), (rvy[0] + (rvy[1]-rvy[0]) * dt), (rvw[0] + (rvw[1]-rvw[0]) * dt)]).transpose()
  # rotation matrix
  # transform reference velocity in global frame to local frame
  cdef double c = cos(theta)
  cdef double s = sin(theta)
  cdef double[9] rot_raw = [c, -s, 0, s, c, 0, 0, 0, 1]
  # cdef np.ndarray[np.float_t, ndim=2] rot = c_rotmat(theta)
  cdef np.ndarray[np.float_t, ndim=2] rot = np.matrix(rot_raw).reshape(3,3)
  # vvec = rot.transpose() * vvec
  vvec = rot.transpose() * vvec
  # return a flattened array
  for i in range(3) : 
    p_ret[i] = pvec[i,0]
    v_ret[i] = vvec[i,0]
  return p_ret, v_ret, i

# compute error term
# takes tuple of double of current pos, ref pos
# i don't think this will have huge effect but let's do it anyway
@cython.boundscheck(False)
@cython.wraparound(False)
def compute_error(np.ndarray[np.float_t, ndim=1] cpos, np.ndarray[np.float_t, ndim=1] rpos) :
  # angle wrap around
  cdef double dw = cpos[2] - rpos[2]
  if fabs(dw) > math.pi :
    dw = (-2*math.pi + dw) if dw > 0 else (2*math.pi + dw)
  return (cpos[0] - rpos[0], cpos[1] - rpos[1], dw)

# compute pi gain matrix from characteristic polynomial
@cython.boundscheck(False)
@cython.wraparound(False)
cpdef compute_from_char_poly(np.ndarray[dtype=np.float_t, ndim=1] pos, np.ndarray[dtype=np.float_t, ndim=1] vel, np.ndarray[dtype=np.float_t, ndim=2] ppoly, np.ndarray[dtype=np.float_t, ndim=2] ipoly) :
  cdef double[9] p_gain
  cdef double[9] i_gain
  cdef double u = vel[0]
  cdef double v = vel[1]
  cdef double c = cos(pos[2])
  cdef double s = sin(pos[2])
  cdef np.ndarray[np.float_t, ndim=2] B = np.matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
  cdef np.ndarray[np.float_t, ndim=2] A = np.matrix([[0, 0, -u*s-v*c], [0, 0, u*c-v*s], [0, 0, 0]])
  cdef np.ndarray[np.float_t, ndim=2] BINV = B.transpose()
  cdef int i=0, j=0, k=0
  cdef double sp = 0.0
  cdef double si = 0.0
  cdef int idx = (i*3)+j
  cdef np.ndarray[np.float_t, ndim=2] kp = BINV * (A-ppoly)
  cdef np.ndarray[np.float_t, ndim=2] ki = -1. * BINV * ipoly
  return (kp, ki)
  # for i in range(3) :
  #   for j in range(3) :
  #     sp = 0.0
  #     si = 0.0
  #     idx = (i*3)+j
  #     for k in range(3) :
  #       si = si + -1 * BINV[i,k] * ipoly[k,j]
  #       sp = sp + BINV[i,k] * (A[k,j] - ppoly[k,j])
  #     p_gain[idx] = sp
  #     i_gain[idx] = si
  # return (np.matrix(p_gain).reshape(3,3), np.matrix(i_gain).reshape(3,3))

@cython.boundscheck(False)
@cython.wraparound(False)
def rotmat(double angle) : 
  return c_rotmat(angle)
