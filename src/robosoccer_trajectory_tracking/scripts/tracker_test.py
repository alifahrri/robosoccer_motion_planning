#!/usr/bin/env python

from cython_tracker import *
import unittest
from math import radians, degrees, cos, sin, pi
import numpy as np

class TestTracker(unittest.TestCase) :
    def test_rotmat(self) : 
        deg_angles = [-180., -150., -120., -90., -45., .0, 45., 90., 120., 150., 180.]
        rad_angles = [radians(x) for x in deg_angles]
        result = [np.matrix([[cos(x), -sin(x), 0],[sin(x), cos(x), 0],[0, 0, 1]]) for x in rad_angles]
        for i in range(len(rad_angles)) :
            r = rotmat(rad_angles[i])
            # check if matrix is same by subtract and see if all member has value > 0.0
            # in other words, if not (or False) any member has value > 0.0
            self.assertFalse((r-result[i]).any(), msg='%s not eq with %s'%(r, result[i]))

    def test_ref_state(self) :
        t = np.array([0., 1., 2., 3.])
        x, vx = np.array([0., 1., 2., 3.]), np.array([1., 1., 1., 1.])
        y, vy = np.array([0., 1., 1., 2.]), np.array([1., 0., 1., 1.])
        w, vw = np.array([0., pi/4., pi/2., 3*pi/4.]), np.array([pi/4., pi/4., pi/4., pi/4.])
        test = [0., 1., 2.]
        res = {
            'pos' : [np.array([0., 0., 0.]), np.array([1., 1., pi/4.]), np.array([2., 1., pi/2.])],
            'vel' : [np.array([1., 1., pi/4.]), np.array([cos(pi/4.), -sin(pi/4.), pi/4.]), np.array([1., -1., pi/4.])]
        }
        for i in range(len(test)) :
            ref = get_ref_state(test[i], x, y, w, vx, vy, vw, t, len(t))
            pos = np.array(ref[0])
            vel = np.array(ref[1])
            self.assertTrue(np.allclose(pos,res['pos'][i]), 'at %s with ref:%s :\nexpect\n%s,\ngot\n%s'%(test[i],ref,res['pos'][i],pos))
            self.assertTrue(np.allclose(vel,res['vel'][i]), 'at %s with ref:%s :\nexpect\n%s,\ngot\n%s'%(test[i],ref,res['vel'][i],vel))

    def test_control(self) :
        t = np.array([0., 1., 2., 3.])
        x, vx = np.array([0., 1., 2., 3.]), np.array([1., 1., 1., 1.])
        y, vy = np.array([0., 1., 1., 2.]), np.array([1., 0., 1., 1.])
        w, vw = np.array([0., pi/4., pi/2., 3*pi/4.]), np.array([pi/4., pi/4., pi/4., pi/4.])
        cpos = np.array([0.1, 0.1, 0.])
        test = [0., 1., 2.]
        ref = get_ref_state(test[0], x, t, w, vx, vy, w, t, len(t))
        rpos, rvel = ref[0], ref[1]
        cmdvel = np.matrix(rvel).reshape(3,1)
        ppoly, ipoly = np.diag([-1., -1., -1.]), np.diag([-1., -1., -1.])
        e = compute_error(cpos, np.array(rpos))
        k = compute_from_char_poly(np.array(rpos), np.array(rvel), ppoly, ipoly)
        pid = {
            'p' : k[0],
            'i' : k[1]
        }
        p_term = pid['p'] * np.matrix([e[0], e[1], e[2]]).transpose()
        # first time, sum of error equals to error
        i_term = pid['i'] * np.matrix([e[0], e[1], e[2]]).transpose()
        c = cmdvel - p_term - i_term
        res = np.matrix([.8,.8,0.]).reshape(3,1)
        self.assertTrue(np.allclose(c,res),'expect %s got %s'%(res, c))
        
    def test_char_poly(self) :
        # this unittest using these characteristic polynomial
        ppoly = np.diag([-.7, -.7, -.7])
        ipoly = np.diag([-.05, -.05, -.05])
        # test case for computing pi gain from charactertisctic polynomial given pos and vel
        test = {
            'pos' : [
                [.0, .0, .0],       [1., .0, .0],       [.0, 1., .0],
                [1., 1., .0],       [-1., .0, .0],      [.0, -1., .0],
                [1., -1., .0],      [-1., 1., .0],      [-1., -1., .0],
                [.0, .0, pi/2.],    [1., 1., pi/2.],    [0., 0., -pi/2]
            ],
            'vel' : [
                [.0, .0, .0],       [1., .0, .0],       [.0, 1., .0],
                [1., 1., .0],       [-1., .0, .0],      [.0, -1., .0],
                [1., -1., .0],      [-1., 1., .0],      [-1., -1., .0],
                [.0, .0, pi/2.],    [1., 1., pi/2.],    [0., 0., -pi/2]
            ]
        }
        # expected result
        res = {
            'KP' : [
                np.matrix([[.7, 0., .0],[.0, .7, .0],[.0, .0, .7]]),
                np.matrix([[.7, 0., .0],[.0, .7, 1.],[.0, .0, .7]]),
                np.matrix([[.7, 0., -1.],[ 0., .7, 0.],[0., 0., .7]]),
                np.matrix([[.7, 0., -1.],[ 0., .7, 1.],[0., 0., .7]]),
                np.matrix([[.7, 0., .0],[0., .7, -1.],[.0, .0, .7]]),
                np.matrix([[.7, 0., 1.],[0., .7, 0.],[0., 0., .7]]),
                np.matrix([[.7, 0., 1.],[0., .7, 1.],[0., 0., .7]]),
                np.matrix([[.7, 0., -1.],[0., .7, -1.],[0., 0., .7]]),
                np.matrix([[.7, 0., 1.],[0., .7, -1.],[0., 0., .7]]),
                np.matrix([[.0, .7, 0.],[-.7, 0., 0.],[0., 0., .7]]),
                np.matrix([[.0, .7, -1.],[-.7, 0., 1.],[0., 0., .7]]),
                np.matrix([[.0, -.7, 0.],[.7, 0., 0.],[0., 0., .7]]),
            ],
            'KI' : [
                np.matrix([[.05, .0, .0],[.0, .05, .0],[.0, .0, .05]]),
                np.matrix([[.05, .0, .0],[.0, .05, .0],[.0, .0, .05]]),
                np.matrix([[.05, .0, .0],[.0, .05, .0],[.0, .0, .05]]),
                np.matrix([[.05, .0, .0],[.0, .05, .0],[.0, .0, .05]]),
                np.matrix([[.05, .0, .0],[.0, .05, .0],[.0, .0, .05]]),
                np.matrix([[.05, .0, .0],[.0, .05, .0],[.0, .0, .05]]),
                np.matrix([[.05, .0, .0],[.0, .05, .0],[.0, .0, .05]]),
                np.matrix([[.05, .0, .0],[.0, .05, .0],[.0, .0, .05]]),
                np.matrix([[.05, .0, .0],[.0, .05, .0],[.0, .0, .05]]),
                np.matrix([[.0, .05, .0],[-.05, .0, .0],[.0, .0, .05]]),
                np.matrix([[.0, .05, .0],[-.05, .0, .0],[.0, .0, .05]]),
                np.matrix([[.0, -.05, .0],[.05, .0, .0],[.0, .0, .05]]),
            ]
        }
        for i in range(len(test['pos'])) :
            pos = test['pos'][i]
            vel = test['vel'][i]
            k = compute_from_char_poly(np.array(pos), np.array(vel), ppoly, ipoly)
            kp = np.matrix(k[0]).reshape(3,3)
            ki = np.matrix(k[1]).reshape(3,3)
            # check if array is near equal
            self.assertTrue(np.allclose(kp,res['KP'][i]), msg='[%s]\nexpect:\n%s, got:\n%s'%(i,res['KP'][i],kp))
            self.assertTrue(np.allclose(ki,res['KI'][i]), msg='[%s]\nexpect:\n%s, got:\n%s'%(i,res['KI'][i],ki))

if __name__ == '__main__' :
    import rosunit
    rosunit.unitrun('robosoccer_trajectory_tracking', 'test_tracker', TestTracker)