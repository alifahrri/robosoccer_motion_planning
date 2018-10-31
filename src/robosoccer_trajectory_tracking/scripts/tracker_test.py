#!/usr/bin/env python

from scripts.tracker import *
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