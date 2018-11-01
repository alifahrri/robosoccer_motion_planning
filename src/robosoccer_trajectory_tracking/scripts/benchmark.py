#!/usr/bin/env python
import tracker
import cytracker
import rospy
import sys
import numpy as np

py_tracker = None
cy_tracker = None

def test_char_poly_cy():
    """Stupid test function"""
    cy_tracker.compute_gain_from_char_poly()

def test_char_poly_py():
    """Stupid test function"""
    py_tracker.compute_gain_from_char_poly()

def test_compute_error_py():
    """Stupid test function"""
    py_tracker.compute_error(50.)

def test_compute_error_cy():
    """Stupid test function"""
    cy_tracker.compute_error(50.)

if __name__ == '__main__':
    rospy.init_node('robosoccer_trajectory_tracking', argv=sys.argv)
    py_tracker = tracker.PITracker()
    cy_tracker = cytracker.PITracker()
    t = np.linspace(0., 50.)
    x = np.linspace(0., 50.)
    y = np.linspace(0., 50.)
    w = np.linspace(0., 50.)
    py_tracker.pos = (0., 0., 0.)
    cy_tracker.pos = (0., 0., 0.)
    cypos, cyvel = cy_tracker.sub['pos'], cy_tracker.sub['vel']
    pypos, pyvel = py_tracker.sub['pos'], py_tracker.sub['vel']
    cypos.t, cypos.x, cypos.y, cypos.w = t, x, y, w
    cyvel.t, cyvel.x, cyvel.y, cyvel.w = t, x, y, w
    pypos.t, pypos.x, pypos.y, pypos.w = t, x, y, w
    pyvel.t, pyvel.x, pyvel.y, pyvel.w = t, x, y, w
    n = int(1e5)
    import timeit
    print('run char_poly python ver %s times : %s'%(n,timeit.timeit("test_char_poly_py()", setup="from __main__ import test_char_poly_py", number=n)))
    print('run char_poly cython ver %s times : %s'%(n,timeit.timeit("test_char_poly_cy()", setup="from __main__ import test_char_poly_cy", number=n)))
    print('run compute_err python ver %s times : %s'%(n,timeit.timeit("test_compute_error_py()", setup="from __main__ import test_compute_error_py", number=n)))
    print('run compute_err cython ver %s times : %s'%(n,timeit.timeit("test_compute_error_cy()", setup="from __main__ import test_compute_error_cy", number=n)))