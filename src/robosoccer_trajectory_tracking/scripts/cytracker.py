import sys
import rospy
# use cython module
from subscriber import *
from scripts.tracker import compute_error, get_ref_state, compute_from_char_poly

import nubot_common.msg as nubotmsg
import numpy as np
import math

class PITracker(RobotSubscriber) :
    def __init__(self, *args, **kwargs):
        agent_id = 1
        all_names = rospy.get_param_names()
        if '/agent_id' in all_names :
            rospy.logwarn('retrieve agent_id from param')
            agent_id = rospy.get_param('/agent_id')
        rospy.logwarn('agent : %s'%agent_id)
        # initialize super class
        RobotSubscriber.__init__(self,'/nubot'+str(agent_id)+'/omnivision/OmniVisionInfo', agent_id)
        ### create nav path subscriber
        self.sub = {
            'pos' : TrajectorySubscriber('robosoccer_trajectory_pos'),
            'vel' : TrajectorySubscriber('robosoccer_trajectory_vel')
        }
        # self.pub = rospy.Publisher('/nubot'+str(agent_id)+'/nubotcontrol/velcmd', nubotmsg.VelCmd, queue_size=3)
        self.pub = rospy.Publisher('/nubot'+str(6)+'/nubotcontrol/velcmd', nubotmsg.VelCmd, queue_size=3)
        self.error = {
            'x' : .0, 'y' : .0, 'w' : .0,
            'sum' : {
                # sum for compute integral part of control
                'x' : .0, 'y' : .0, 'w' : .0,
            }
        }
        self.control = {
            'x' : .0, 'y' : .0, 'w' : .0,
            'com' : np.array([0., 0., 0.])
        }
        self.pid = {
            # some tuning parameter
            'p' : np.diag([1., 1., 1.]), 
            'i' : np.diag([.2, .2, .2])
        }
        # characteristic polynomial for computing gain
        self.char_poly = {
            'p' : [1., 1., 1.], 
            'i' : [.2, .2, .2]
        }
        # command (or reference if you like)
        self.command = {
            # 'pos' : {'x' : .0, 'y' : .0, 'w' : .0,},
            # 'vel' : {'x' : .0, 'y' : .0, 'w' : .0,}
            'pos' : np.array([0., 0., 0.]),
            'vel' : np.array([0., 0., 0.]),
        }
        rospy.logwarn('READY')

    def compute_pi_from_char_poly(self) :
        t0 = rospy.get_time()
        # pos = np.array([self.command['pos'][k] for k in ['x', 'y', 'w']])
        # vel = np.array([self.command['vel'][k] for k in ['x', 'y', 'w']])
        pos, vel = self.command['pos'], self.command['vel']
        ppoly = np.array(self.char_poly['p'])
        ipoly = np.array(self.char_poly['i'])
        k = compute_from_char_poly(pos, vel, ppoly, ipoly)
        self.pid['p'] = k[0]
        self.pid['i'] = k[1]
        rospy.loginfo('computed gain in %s s'%(rospy.get_time() - t0))

    def compute_control(self) :
        t0 = rospy.get_time()
        c, pid, e = self.control, self.pid, self.error
        s = e['sum']
        p_term = pid['p'] * np.matrix([[e['x']], [e['x']], [e['w']]])
        i_term = pid['i'] * np.matrix([[s['x']], [s['y']], [s['w']]])
        c['x'] = c['com'][0] - p_term[0] - i_term[0]
        c['y'] = c['com'][1] - p_term[1] - i_term[1]
        c['w'] = c['com'][2] - p_term[2] - i_term[2]
        rospy.loginfo('computed control in %s s'%(rospy.get_time() - t0))

    def compute_error(self, time) :
        sub = self.sub
        idx = 0
        l = len(sub['pos'].t)
        # if ref is empty we can't do anything
        if l < 1 : return False
        # measure time
        t0 = rospy.get_time()
        pos, vel = sub['pos'], sub['vel']
        # use cython module for compute reference (or desired) state and compute the error
        ref = get_ref_state(time, np.array(pos.x), np.array(pos.y), np.array(pos.w), np.array(vel.x), np.array(vel.y), np.array(vel.w), np.array(pos.t))
        e = compute_error(np.array(self.pos), ref[0])
        error, control, command = self.error, self.control, self.command
        command['pos'] = np.array(ref[0])
        command['vel'] = np.array(ref[1])
        # command['pos'] = {'x' : ref[0][0], 'y' : ref[0][1], 'w' : ref[0][2] }
        # command['vel'] = {'x' : ref[1][0], 'y' : ref[1][1], 'w' : ref[1][2] }
        control['com'] = command['vel']
        error['x'], error['y'], error['w'] = e[0], e[1], e[2]
        sum = error['sum']
        sum['x'] = sum['x'] + error['x']
        sum['y'] = sum['y'] + error['y']
        sum['w'] = sum['w'] + error['w']
        rospy.loginfo('computed error in %s s'%(rospy.get_time()-t0))
        return True

    def publish(self) :
        e, c, p = self.error, self.control, self.pid
        s = e['sum']
        vel = nubotmsg.VelCmd()
        vel.Vx = self.control['x']
        vel.Vy = self.control['y']
        vel.w = self.control['w']
        self.pub.publish(vel)
        # rospy.loginfo('pi:(%s,%s)'%(p['p'],p['i']))
        # rospy.logwarn('E:(%s,%s,%s)'%(e['x'],e['y'],e['w']))
        # rospy.logwarn('S:(%s,%s,%s)'%(s['x'],s['y'],s['w']))
        # rospy.logwarn('C:(%s,%s,%s)'%(c['x'],c['y'],c['w']))

    def callback(self, msg) :
        rospy.loginfo('pi_tracker update :')
        super(PITracker, self).callback(msg)
        time = self.header.stamp.to_sec()
        if self.compute_error(time) :
            self.compute_pi_from_char_poly()
            self.compute_control()
            self.publish()
