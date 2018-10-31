# ros stuff
import tf
import rospy
from nav_msgs import msg as navmsg
import std_msgs
import control_msgs
import geometry_msgs as geomsg

import nubot_common.msg as msg

class RobotSubscriber(object) :
  def __init__(self, topic, id, *args, **kwargs):
    self.sub = rospy.Subscriber(topic,msg.OminiVisionInfo,callback=self.callback)
    self.id = id
    self.pos, self.vel = None, None
    self.header = None

  def callback(self, info) :
    rospy.loginfo('receive message')
    for robot in info.robotinfo :
      self.header = info.header
      if robot.AgentID == self.id :
        self.pos = (robot.pos.x/100.0, robot.pos.y/100.0, robot.heading.theta)
        self.vel = (robot.vtrans.x/100.0, robot.vtrans.y/100.0, robot.vrot)
        break

class GoalSubscriber() :
  def __init__(self, *args, **kwargs):
    self.sub = rospy.Subscriber('/move_base_simple/goal', geomsg.msg.PoseStamped, callback=self.goal_callback)
    self.goal = geomsg.msg.PoseStamped()
 
  def goal_callback(self, msg) :
    self.goal = msg

class TrajectorySubscriber(object) :
  def __init__(self, topic, *args, **kwargs) :
    self.subscriber = rospy.Subscriber(topic, navmsg.Path, callback=self.callback, queue_size=3)
    self.t, self.x, self.y, self.w = [], [], [], []

  def callback(self, msg) :
    rospy.loginfo('receive message')
    t, x, y, w = [], [], [], []
    for i in range(len(msg.poses)) :
      pt = msg.poses[i]
      p = pt.pose.position
      o = pt.pose.orientation
      quaternion = (o.x,o.y,o.z,o.w)
      euler = tf.transformations.euler_from_quaternion(quaternion)
      yaw = euler[2]
      t.append(pt.header.stamp.to_sec())
      x.append(p.x)
      y.append(p.y)
      w.append(yaw)
    self.t, self.x, self.y, self.w = t, x, y, w