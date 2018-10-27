#!/usr/bin/python
import sys
import rospy
from subscriber import *
import nubot_common.msg as nubotmsg

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
            'p' : 1., 'i' : .2
        }
        rospy.logwarn('READY')

    def compute_error(self, time) :
        sub = self.sub
        idx = 0
        l = len(sub['pos'].t)
        # if ref is empty we can't do anything
        if l < 1 : return False
        # if indexed time behid reference, take first element
        # if beyond last ref, take the last
        # iterate if in between
        if time < sub['pos'].t[0] : idx = 0
        elif time > sub['pos'].t[-1] : idx = l-1 
        else :
            for i in range(l) :
                if i == 0 : continue
                idx = i
        i = idx
        t = (sub['pos'].t[i], sub['pos'].t[i-1])
        if (time > t[0]) and (time < t[1]) :
            dt = time - t[0]
            # get reference
            pos, vel = sub['pos'], sub['vel']
            rx = (pos.x[i], pos.x[i-1])
            ry = (pos.y[i], pos.y[i-1])
            rw = (pos.w[i], pos.w[i-1])
            rvx = (vel.x[i], vel.x[i-1])
            rvy = (vel.y[i], vel.y[i-1])
            rvw = (vel.w[i], vel.w[i-1])
            # compute error
            ## get computed error
            sum, ex = self.error['sum'], self.error['x']
            ey, ew = self.error['y'], self.error['w']
            self.control['x'] = (rvx[0] + (rvx[1]-rvx[0]) * dt)
            self.control['y'] = (rvy[0] + (rvy[1]-rvy[0]) * dt)
            self.control['w'] = (rvw[0] + (rvw[1]-rvw[0]) * dt)
            ex = self.pos[0] - (rx[0] + (rx[1]-rx[0]) * dt)
            ey = self.pos[1] - (ry[0] + (ry[1]-ry[0]) * dt)
            ## TODO : fix angle shortest path!!!
            ew = self.pos[2] - (rw[0] + (rw[1]-rw[0]) * dt)
            # sum the error
            sum['x'] = sum['x'] + ex
            sum['y'] = sum['y'] + ey
            sum['w'] = sum['w'] + ew
            return True
        return False

    def compute_control(self) :
        for k in self.control.keys() :
            self.control[k] = self.pid['p'] * self.error[k] + self.pid['i'] * self.error['sum'][k]

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
        if self.compute_error(time) :
            self.compute_control()
            self.publish()

if __name__ == '__main__' :
    rospy.init_node('robosoccer_trajectory_tracking', argv=sys.argv)
    tracker = PITracker()
    rospy.spin()