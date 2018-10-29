import sys
import rospy
from subscriber import *
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
        self.pub = rospy.Publisher('/nubot'+str(agent_id)+'/nubotcontrol/velcmd', nubotmsg.VelCmd, queue_size=3)
        self.error = {
            'x' : .0, 'y' : .0, 'w' : .0,
            'sum' : {
                # sum for compute integral part of control
                'x' : .0, 'y' : .0, 'w' : .0,
            }
        }
        self.control = {
            'x' : .0, 'y' : .0, 'w' : .0,
            'com' : {
                # commanded velocity
                'x' : .0, 'y' : .0, 'w' : .0,
            }
        }
        self.pid = {
            # some tuning parameter
            'p' : np.diag([1., 1., 1.]), 
            'i' : np.diag([.2, .2, .2])
        }
        # characteristic polynomial for computing gain
        self.char_poly = {
            'p' : np.diag([1., 1., 1.]), 
            'i' : np.diag([.2, .2, .2])
        }
        # command (or reference if you like)
        self.command = {
            'pos' : {'x' : .0, 'y' : .0, 'w' : .0,},
            'vel' : {'x' : .0, 'y' : .0, 'w' : .0,}
        }
        rospy.logwarn('READY')

    def compute_pi_from_char_poly(self) :
        t0 = rospy.get_time()
        pos, vel = self.command['pos'], self.command['vel']
        poly = self.char_poly
        c, s = math.cos(pos['w']), math.cos(pos['w'])
        u, v = vel['x'], vel['y']
        B = np.matrix([[c, -s, 0],[s, c, 0],[0, 0, 1]])
        A = np.matrix([[0, 0, -u*s-v*c],[0, 0, u*c-v*s],[0, 0, 0]])
        self.pid = {
            'p' : np.linalg.inv(-1 * B) * poly['p'],
            'i' : np.linalg.inv(B) * (A - poly['i'])
        }
        rospy.loginfo('computed gain in %s s'%(rospy.get_time() - t0))
    
    def compute_control(self) :
        t0 = rospy.get_time()
        c, pid, e = self.control, self.pid, self.error
        s = e['sum']
        p_term = pid['p'] * np.matrix([[e['x']], [e['x']], [e['w']]])
        i_term = pid['i'] * np.matrix([[s['x']], [s['y']], [s['w']]])
        c['x'] = c['com']['x'] + p_term[0] + i_term[0]
        c['y'] = c['com']['y'] + p_term[1] + i_term[1]
        c['w'] = c['com']['w'] + p_term[2] + i_term[2]
        rospy.loginfo('computed control in %s s'%(rospy.get_time() - t0))
        # for k in set(control.keys()).intersection(error.keys()) :
        #     control[k] = control['com'][k] + pid['p'] * error[k] + pid['i'] * error['sum'][k]

    def compute_error(self, time) :
        sub = self.sub
        idx = 0
        l = len(sub['pos'].t)
        # if ref is empty we can't do anything
        if l < 1 : return False
        t0 = rospy.get_time()
        pos, vel = sub['pos'], sub['vel']
        # if indexed time behid reference, take first element
        # if beyond last ref, take the last
        # iterate if in between
        rx = (pos.x[0], pos.x[0])
        ry = (pos.y[0], pos.y[0])
        rw = (pos.w[0], pos.w[0])
        rvx = (vel.x[0], vel.x[0])
        rvy = (vel.y[0], vel.y[0])
        rvw = (vel.w[0], vel.w[0])
        if time < sub['pos'].t[0] : idx = 0
        elif time > sub['pos'].t[-1] : 
            idx = l-1 
            rx = (pos.x[idx], pos.x[idx])
            ry = (pos.y[idx], pos.y[idx])
            rw = (pos.w[idx], pos.w[idx])
            rvx = (vel.x[idx], vel.x[idx])
            rvy = (vel.y[idx], vel.y[idx])
            rvw = (vel.w[idx], vel.w[idx])
        else :
            for i in range(l) :
                if i == 0 : continue
                t = (sub['pos'].t[i-1], sub['pos'].t[i])
                if (time > t[0]) and (time < t[1]) :
                    idx = i
                    rx = (pos.x[i-1], pos.x[i])
                    ry = (pos.y[i-1], pos.y[i])
                    rw = (pos.w[i-1], pos.w[i])
                    rvx = (vel.x[i-1], vel.x[i])
                    rvy = (vel.y[i-1], vel.y[i])
                    rvw = (vel.w[i-1], vel.w[i])
                    break
        i = idx
        t = (sub['pos'].t[i-1], sub['pos'].t[i])
        if i==0 : t = (sub['pos'].t[0], sub['pos'].t[0])
        if (time > t[0]) and (time < t[1]) :
            dt = time - t[0]
            ref = self.command
            # save reference trajectory
            ref['vel'] = {
                'x' : (rvx[0] + (rvx[1]-rvx[0]) * dt),
                'y' : (rvy[0] + (rvy[1]-rvy[0]) * dt),
                'w' : (rvw[0] + (rvw[1]-rvw[0]) * dt)
            }
            ref['pos'] = {
                'x' : (rx[0] + (rx[1]-rx[0]) * dt),
                'y' : (ry[0] + (ry[1]-ry[0]) * dt),
                'w' : (rw[0] + (rw[1]-rw[0]) * dt)
            }
            self.control['com'] = self.command['vel']
            # compute error
            ## get computed error
            sum, ex = self.error['sum'], self.error['x']
            ey, ew = self.error['y'], self.error['w']
            ex = self.pos[0] - ref['pos']['x']
            ey = self.pos[1] - ref['pos']['y']
            ## TODO : fix angle shortest path!!!
            ew = self.pos[2] - ref['pos']['w']
            # sum the error for integral term
            sum['x'] = sum['x'] + ex
            sum['y'] = sum['y'] + ey
            sum['w'] = sum['w'] + ew
            rospy.loginfo('computed error in %s s'%(rospy.get_time()-t0))
            return True
        return False

    def publish(self) :
        e = self.error
        c = self.control
        p = self.pid
        s = e['sum']
        rospy.loginfo('pi:(%s,%s)'%(p['p'],p['i']))
        rospy.logwarn('E:(%s,%s,%s)'%(e['x'],e['y'],e['w']))
        rospy.logwarn('S:(%s,%s,%s)'%(s['x'],s['y'],s['w']))
        rospy.logwarn('C:(%s,%s,%s)'%(c['x'],c['y'],c['w']))
    
    def callback(self, msg) :
        rospy.loginfo('pi_tracker update :')
        super(PITracker, self).callback(msg)
        time = self.header.stamp.to_sec()
        self.compute_error(time)
        self.compute_pi_from_char_poly()
        # if self.compute_error(time) :
        #     self.compute_pi_from_char_poly()
        #     self.compute_control()
        #     self.publish()