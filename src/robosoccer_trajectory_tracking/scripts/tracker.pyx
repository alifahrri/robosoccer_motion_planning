#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy

import nubot_common.msg as msg

import numpy as np

cimport cython
cimport numpy as np

# compute matrix ki and kp from characteristic polynomial,
# given error, sum of error, command (pos & vel)
# return tuple of matrix
cdef ccompute_from_char_poly(np.ndarray[dtype=np.float_t] pos, np.ndarray[dtype=np.float_t] vel, np.ndarray[dtype=np.float_t] ppoly, np.ndarray[dtype=np.float_t] ipoly) :
  cdef double c, s, u, v
  c, s = math.cos(pos[2]), math.sin(pos[2])
  u, v = vel[0], vel[1]
  cdef double[9] braw = [c, -s, 0, s, c, 0, 0, 0, 1]
  cdef double[9] araw = [0, 0, -u*s-v*c, 0, 0, c*c-v*s, 0, 0, 0]
  cdef np.ndarray[np.float_t, ndim=2] B = np.array(braw).reshape(3,3)
  cdef np.ndarray[np.float_t, ndim=2] A = np.array(araw).reshape(3,3)
  cdef np.ndarray[np.float_t, ndim=2] P = np.diag(ppoly)
  cdef np.ndarray[np.float_t, ndim=2] I = np.diag(ipoly)
  cdef np.ndarray[np.float_t, ndim=2] BINV = B.transpose()
  # cdef np.ndarray[np.float_t, ndim=2] BINV = np.linalg.inv(B)
  # cdef np.ndarray[np.float_t, ndim=2] p
  # cdef np.ndarray[np.float_t, ndim=2] i
  # p = -1*BINV*P
  # i = BINV*(A - I)
  cdef double[9] praw
  cdef double[9] iraw
  cdef int i=0, j=0, k=0
  cdef double sp = 0.0
  cdef double si = 0.0
  cdef int idx = (i*3)+j
  for i in range(3) :
    for j in range(3) :
      sp = 0.0
      si = 0.0
      idx = (i*3)+j
      for k in range(3) :
        sp = sp + -1 * BINV.flat[(i*3)+k] * P.flat[(k*3)+j]
        si = si + BINV.flat[(i*3)+k] * (A.flat[(k*3)+j]-I.flat[(k*3)+j])
      praw[idx] = sp
      iraw[idx] = si
  cdef np.ndarray[np.float_t, ndim=2] pgain = np.array(praw).reshape(3,3)
  cdef np.ndarray[np.float_t, ndim=2] igain = np.array(iraw).reshape(3,3)
  return  (pgain, igain)

@cython.boundscheck(False)
@cython.wraparound(False)
def compute_from_char_poly(np.ndarray[dtype=np.float_t, ndim=1] pos, np.ndarray[dtype=np.float_t, ndim=1] vel, np.ndarray[dtype=np.float_t, ndim=1] ppoly, np.ndarray[dtype=np.float_t, ndim=1] ipoly) :
  return ccompute_from_char_poly(pos, vel, ppoly, ipoly)

# find reference state
# given current time, vector of x, y, w, vx (or u), vy (or v), vw (or r), adn t
# returning tuple of pos (x,y,z) and vel (x,y,z)
cdef cget_ref_state(double time, np.ndarray[np.float_t, ndim=1] x, np.ndarray[np.float_t, ndim=1] y, np.ndarray[np.float_t, ndim=1] w, np.ndarray[np.float_t, ndim=1] u, np.ndarray[np.float_t, ndim=1] v, np.ndarray[np.float_t, ndim=1] r, np.ndarray[np.float_t, ndim=1] t) :
  rx, ry, rw = (x[0], x[0]), (y[0], y[0]), (w[0], w[0])
  rvx, rvy, rvw = (u[0], u[0]), (v[0], v[0]), (r[0], r[0])
  cdef double dt
  cdef int i
  cdef int l
  dt = time - t[0]
  if time < t[0] : pass
  elif time > t[-1] :
    rx, ry, rw = (x[-1], x[-1]), (y[-1], y[-1]), (w[-1], w[-1])
    rvx, rvy, rvw = (u[-1], u[-1]), (v[-1], v[-1]), (r[-1], r[-1])
  else :
    l = len(t)
    for i in range(l) :
      if i == 0 : continue
      elif (time > t[i-1]) and (time < t[i]) :
        rx, ry, rw = (x[i-1], x[i]), (y[i-1], y[i]), (w[i-1], w[i])
        rvx, rvy, rvw = (u[i-1], u[i]), (v[i-1], v[i]), (r[i-1], r[i])
        break
  # cdef (double, double, double) vel
  # cdef (double, double, double) pos
  # cdef double[3] pos
  # cdef double[3] vel
  cdef np.ndarray[np.float_t, ndim=1] pvec
  cdef np.ndarray[np.float_t, ndim=1] vvec
  pvec = np.array([(rx[0] + (rx[1]-rx[0]) * dt), (ry[0] + (ry[1]-ry[0]) * dt), (rw[0] + (rw[1]-rw[0]) * dt)]).reshape(3,1)
  vvec = np.array([(rvx[0] + (rvx[1]-rvx[0]) * dt), (rvy[0] + (rvy[1]-rvy[0]) * dt), (rvw[0] + (rvw[1]-rvw[0]) * dt)]).reshape(3,1)
  # rotation matrix
  cdef double c = math.cos(pvec[2])
  cdef double s = math.sin(pvec[2])
  cdef double[9] rot_raw = [c, s, 0, -s, c, 0, 0, 0, 1]
  # transform reference velocity in global frame to local frame
  cdef np.ndarray[np.float_t, ndim=2] rot = np.array(rot_raw).reshape(3,3)
  vvec = rot * vvec
  return (pvec, vvec)

# def get_ref_state(double time, double[:] x, double[:] y, double[:] w, double[:] u, double[:] v, double[:] r, double[:] t) :
@cython.boundscheck(False)
@cython.wraparound(False)
def get_ref_state(double time, np.ndarray[np.float_t, ndim=1] x, np.ndarray[np.float_t, ndim=1] y, np.ndarray[np.float_t, ndim=1] w, np.ndarray[np.float_t, ndim=1] u, np.ndarray[np.float_t, ndim=1] v, np.ndarray[np.float_t, ndim=1] r, np.ndarray[np.float_t, ndim=1] t) :
  return cget_ref_state(time, x, y, w, u, v, r, t)

# compute error term
# takes tuple of double of current pos, ref pos
@cython.boundscheck(False)
@cython.wraparound(False)
def compute_error(np.ndarray cpos, (double,double,double) rpos) :
  return (cpos[0] - rpos[0], cpos[1] - rpos[1], cpos[2] - rpos[2])