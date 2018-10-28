# ros stuff
import tf
import rospy
from std_msgs.msg import Header
import control_msgs
import geometry_msgs

# import nav_msgs.msg as navmsg
# import nubot_common.msg as msg
from nav_msgs.msg import Path
from nubot_common.msg import OminiVisionInfo

cdef class RobotSubscriber :
  def __init__(self, topic, id):
    self.sub = rospy.Subscriber(topic,OminiVisionInfo,callback=self.callback)
    self.id = id
    self.pos, self.vel = None, None
    self.header = Header()

  def callback(self, info) :
    rospy.loginfo('receive message')
    for robot in info.robotinfo :
      self.header = info.header
      if robot.AgentID == self.id :
        self.pos = (robot.pos.x/100.0, robot.pos.y/100.0, robot.heading.theta)
        self.vel = (robot.vtrans.x/100.0, robot.vtrans.y/100.0, robot.vrot)
        break

class TrajectorySubscriber :
  def __init__(self, topic) :
    self.subscriber = rospy.Subscriber(topic, Path, callback=self.callback, queue_size=3)
    self.t, self.x, self.y, self.w = [], [], [], []

  def callback(self, message) :
    rospy.loginfo('receive message')
    cdef list t = []
    cdef list x = []
    cdef list y = []
    cdef list w = [] 
    cdef int i
    cdef int l 
    l = len(message.poses)
    for i in range(l) :
      pt = message.poses[i]
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