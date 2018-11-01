import sys
import rospy
import rosparam
# use cython module
from subscriber import *
from cython_tracker import compute_error, get_ref_state, compute_from_char_poly

import nubot_common.msg as nubotmsg
import robosoccer_trajectory_tracking.msg as trackmsg
import numpy as np
import math
import yaml

class PITracker(RobotSubscriber, GoalSubscriber) :
    def __init__(self, *args, **kwargs):
        agent_id = 1
        all_names = rospy.get_param_names()
        if '/agent_id' in all_names :
            rospy.logwarn('retrieve agent_id from param')
            agent_id = rospy.get_param('/agent_id')
        rospy.logwarn('agent : %s'%agent_id)
        # initialize super class
        GoalSubscriber.__init__(self)
        RobotSubscriber.__init__(self,'/nubot'+str(agent_id)+'/omnivision/OmniVisionInfo', agent_id)
        ### create nav path subscriber
        self.sub = {
            'pos' : TrajectorySubscriber('robosoccer_trajectory_pos'),
            'vel' : TrajectorySubscriber('robosoccer_trajectory_vel')
        }
        self.sub['pos'].register_callback(self.trajectory_callback)
        # self.pub = rospy.Publisher('/nubot'+str(agent_id)+'/nubotcontrol/velcmd', nubotmsg.VelCmd, queue_size=3)
        self.pub = rospy.Publisher('/nubot'+str(agent_id)+'/nubotcontrol/velcmd', nubotmsg.VelCmd, queue_size=3)
        self.info_pub = rospy.Publisher('/tracker_info', trackmsg.ControlInfo, queue_size=3)
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
            # initial value, will be overwritten
            'p' : np.diag([1., 1., 1.]), 
            'i' : np.diag([.05, .05, .05])
        }
        # characteristic polynomial for computing gain
        ## default value
        poly = {
            'p' : [.5, .5, .5],
            'i' : [.05, .05, .05]
        }
        ## check if we have in rosparam
        params = rosparam.list_params(rospy.get_name())
        rospy.loginfo('parameter in %s : %s'%(rospy.get_name(), params))
        if any('char_poly' in x for x in params) :
            rospy.logwarn('loading char poly from param')
            poly = rosparam.get_param(rospy.get_name()+'/char_poly')
            rospy.logwarn('poly : %s'%poly)
        try :
            self.enable_tracking = rosparam.get_param(rospy.get_name()+'/enable')
            rospy.logwarn('tracking enable : %s' %self.enable_tracking)
        except : 
            rospy.logwarn('tracking not set, default to false')
            self.enable_tracking = False
        self.char_poly = {
            # some tuning parameter
            # 'p' : np.diag([0., 0., 0.]), 
            # 'i' : np.diag([0., 0., 0.])
            'p' : np.diag(poly['p']) * -1.,
            'i' : np.diag(poly['i']) * -1.,
            # 'p' : np.diag([-.5, -.5, -.5]), 
            # 'i' : np.diag([-.05, -.05, -.05])
        }
        # command (or reference if you like)
        self.command = {
            # 'pos' : {'x' : .0, 'y' : .0, 'w' : .0,},
            # 'vel' : {'x' : .0, 'y' : .0, 'w' : .0,}
            'pos' : np.array([0., 0., 0.]),
            'vel' : np.array([0., 0., 0.]),
        }
        self.last_update = None
        # this enable is for waiting first goal to be published
        self.enable = False
        # saturation
        self.saturate = {
            'enabled' : True,
            'saturated' : False,
            'limit' : {
                'trans' : 3., 'rot' : 1.5
            }
        }
        self.ref_time = rospy.get_time()
        self.ctrl_time = rospy.get_time()
        rospy.logwarn('READY')

    def compute_gain_from_char_poly(self) :
        # t0 = rospy.get_time()
        pos, vel = self.command['pos'], self.command['vel']
        poly = self.char_poly
        k = compute_from_char_poly(pos, vel, poly['p'], poly['i'])
        self.pid = {
            'p' : k[0],
            'i' : k[1]
        }
        # rospy.loginfo('computed gain in %s s'%(rospy.get_time() - t0))

    def compute_control(self) :
        # t0 = rospy.get_time()
        self.compute_gain_from_char_poly()
        c, pid, e = self.control, self.pid, self.error
        s = e['sum']
        p_term = pid['p'] * np.matrix([e['x'], e['y'], e['w']]).transpose()
        i_term = pid['i'] * np.matrix([s['x'], s['y'], s['w']]).transpose()
        if self.enable_tracking :
            c['x'] = c['com'][0] - p_term[0] - i_term[0]
            c['y'] = c['com'][1] - p_term[1] - i_term[1]
            c['w'] = c['com'][2] - p_term[2] - i_term[2]
        else :
            c['x'] = c['com'][0]
            c['y'] = c['com'][1]
            c['w'] = c['com'][2]
        if self.saturate['enabled'] :
            limit = self.saturate['limit'] 
            saturated = False
            if math.hypot(c['x'],c['y']) > limit['trans'] :
                angle = math.atan2(c['y'],c['x'])
                c['x'] = math.cos(angle) * limit['trans']
                c['y'] = math.sin(angle) * limit['trans']
                saturated = True
            if math.fabs(c['w']) > limit['rot'] :
                c['w'] = -limit['rot'] if c['w'] < 0. else limit['rot']
            self.saturate['saturated'] = saturated
        # rospy.loginfo('computed control in %s s'%(rospy.get_time() - t0))

    def get_ref_state(self, time) :
        sub = self.sub
        l = len(sub['pos'].t)
        # if ref is empty we can't do anything
        if l < 1 : return None
        # measure time
        # t0 = rospy.get_time()
        pos, vel = sub['pos'], sub['vel']
        # use cython module for compute reference (or desired) state and compute the error
        ref = get_ref_state(time, np.array(pos.x), np.array(pos.y), np.array(pos.w), np.array(vel.x), np.array(vel.y), np.array(vel.w), np.array(pos.t), l)
        return ref

    def compute_error(self, time) :
        ref = self.get_ref_state(time)
        if ref is None : return False
        e = compute_error(np.array(self.pos), np.array(ref[0]))
        error, control, command = self.error, self.control, self.command
        command['pos'] = np.array(ref[0])
        command['vel'] = np.array(ref[1])
        control['com'] = command['vel']
        error['x'], error['y'], error['w'] = e[0], e[1], e[2]
        sum = error['sum']
        dt = self.dt
        # sum_en = True
        # if we use saturation, don't sum if sum of error already saturated
        if self.saturate['enabled'] and self.saturate['saturated'] :
            for k in ('x', 'y', 'w') :
                ds = sum[k] + error[k] * dt
                sum[k] = ds if math.fabs(ds) < math.fabs(sum[k]) else sum[k]
        else :
            for k in ['x', 'y', 'w'] :
                sum[k] = sum[k] + error[k] * dt
        # rospy.loginfo('computed error in %s s'%(rospy.get_time()-t0))
        return True

    def publish_control_info(self) :
        if self.enable :
            ctrl_info = trackmsg.ControlInfo()
            key = ('x', 'y', 'w')
            for k in key :
                ctrl_info.info.append(self.error[k])
            for k in key :
                ctrl_info.info.append(self.error['sum'][k])
            self.info_pub.publish(ctrl_info)

    def publish(self) :
        e, c, k = self.error, self.control, self.pid
        s = e['sum']
        vel = nubotmsg.VelCmd()
        # nubot use cm/s
        vel.Vx, vel.Vy, vel.w = c['x']*100.0, c['y']*100.0, c['w']
        p, i = k['p'], k['i']
        rospy.logwarn('ref time : %s'%self.ref_time)
        rospy.logwarn('control time : %s'%self.ctrl_time)
        rospy.logwarn('command\t:(%s,%s,%s)'%(c['com'][0],c['com'][1],c['com'][2]))
        rospy.logwarn('gain:\nKP\n:%s\nKI:\n%s)'%(p,i))
        rospy.logwarn('error\t:(%s,%s,%s)'%(e['x'],e['y'],e['w']))
        rospy.logwarn('sum of error\t:(%s,%s,%s)'%(e['sum']['x'],e['sum']['y'],e['sum']['w']))
        rospy.logwarn('control\t:(%s,%s,%s)'%(c['x'],c['y'],c['w']))
        # only publish when enabled, which means goal message received
        if self.enable :
            self.pub.publish(vel)
        self.publish_control_info()

    def callback(self, msg) :
        time = rospy.get_time()
        rospy.loginfo('pi_tracker update :')
        super(PITracker, self).callback(msg)
        # use current time
        self.dt = time - self.ctrl_time
        # compute error for the current time
        if self.compute_error(time) :
            # the time when reference received
            self.ref_time = self.sub['pos'].t[0]
            # current time
            self.compute_control()
            self.publish()
            self.ctrl_time = time
    
    def trajectory_callback(self, t, x, y, w) :
        # reset error when new trajectory is received
        self.error = {
            'x' : .0, 'y' : .0, 'w' : .0,
            'sum' : {
                # sum for compute integral part of control
                'x' : .0, 'y' : .0, 'w' : .0,
            }
        }

    def goal_callback(self, msg) :
        rospy.loginfo('pi_tracker goal callback :')
        super(PITracker, self).goal_callback(msg)
        # reset error when goal is changed
        self.error = {
            'x' : .0, 'y' : .0, 'w' : .0,
            'sum' : {
                # sum for compute integral part of control
                'x' : .0, 'y' : .0, 'w' : .0,
            }
        }
        self.enable = True
