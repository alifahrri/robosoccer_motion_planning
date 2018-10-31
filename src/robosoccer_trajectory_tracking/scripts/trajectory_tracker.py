#!/usr/bin/env python
from tracker import *

if __name__ == '__main__' :
    rospy.init_node('robosoccer_trajectory_tracking', argv=sys.argv)
    tracker = PITracker()
    rospy.spin()