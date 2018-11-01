import rospy
import math
import tf
import nubot_common.msg as msg
import geometry_msgs.msg as geomsg

class VelocityPublisher(object) :
    def __init__(self, topic, *args, **kwargs):
        self.publisher = rospy.Publisher(topic,msg.VelCmd,queue_size=1)
        self.goal_pub = None
        self.msg = msg.VelCmd()
        self.enabled, self.move_base = False, False
        self.key = 'stop'
        self.angle, self.angle_rate, self.vmax = 0.0, 5.0, 1.0
        self.last_target = None
        self.last_error = (None, None)
        self.pd_gain = {
            'trans' : {
                'p' : 1.0,
                'd' : 2.0
            },
            'rot' : {
                'p' : 1.0,
                'd' : 2.0
            }
        }

    def set_gain(self, trans, rot) :
        # assuming dictionary
        self.pd_gain['trans'] = trans
        self.pd_gain['rot'] = rot
    
    def publish(self) :
        vel = self.msg
        if not self.enabled :
            return
        elif self.key == 'stop' :
            vel.Vx = 0.0
            vel.Vy = 0.0
            vel.w = 0.0
        elif self.key == 'circular' :
            vm = self.vmax
            angle = math.radians(self.angle)
            vel.Vx = math.cos(angle)*vm
            vel.Vy = math.sin(angle)*vm
            vel.w = 0.0
            self.angle = self.angle + self.angle_rate
        # only publish velocity if not use move base
        if self.key == 'move_base' :
            if self.move_base :
                self.publishGoal(self.last_target)
                self.move_base = False
        else : self.publisher.publish(vel)

    def publishGoal(self, target) :
        goal = geomsg.PoseStamped()
        goal.pose.position.x = target[0] / 100.
        goal.pose.position.y = -target[1] / 100.
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, -target[2])
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        self.goal_pub.publish(goal)

    def error(self, target, pose) :
        angle = pose[2]
        c, s = math.cos(angle), math.sin(angle)
        dx = target[0] - pose[0]
        dy = target[1] - pose[1]
        dw = -target[2] - pose[2]
        if math.fabs(dw) > math.pi :
            dw -= math.copysign(math.pi, -dw)
        lx, ly = c*dx - s*dy, c*dy + s*dx
        error = math.hypot(lx, ly)
        lw = math.atan2(-ly, lx)
        return error, dw, lw, dx, dy

    def pidControl(self, target, pose) :
        if (self.key != 'waypoint') or (target is None) or (pose is None) :
            return None
        if not (self.last_target is None) :
            if target != self.last_target :
                self.last_error = (None, None)
                self.last_target = target
        # param
        gain = self.pd_gain
        MAX_VTRANS, MAX_VROT = 200.0, 1.0
        KP_TRANS, KD_TRANS = gain['trans']['p'], gain['trans']['d']
        KP_ROT, KD_ROT = gain['rot']['p'], gain['trans']['d']
        # compute error
        error, dw, lw, dx, dy = self.error(target, pose)
        lc, ls = math.cos(lw), math.sin(lw)
        d_error, d_error_rot = 0.0, 0.0
        if not (self.last_error[0] is None) :
            d_error = error - self.last_error[0] 
            d_error_rot = dw - self.last_error[1]
        V = KP_TRANS * error + KD_TRANS * d_error
        w = KP_ROT * dw + KD_ROT * d_error_rot
        if V > MAX_VTRANS : V = MAX_VTRANS
        self.msg.Vx = lc * V
        self.msg.Vy = ls * V
        self.msg.w = w
        self.last_error = (error, dw)
        return self.last_error

    def control(self, target, pose) :
        if self.key == 'move_base' :
            return self.moveBase(target, pose)
        else : 
            return self.pidControl(target, pose)

    def moveBase(self, target, pose) :
        error = None
        if (self.key != 'move_base') or (target is None) or (pose is None) or (not self.enabled):
            return error
        elif not (self.last_target is None) :
            if target != self.last_target :
                self.last_error = (None, None)
                self.last_target = target
                self.move_base = True
                # self.publishGoal(target)
            else :
                e = self.error(target, pose)
                error = [e[0]]
        else : # first time, publish goal
            e = self.error(target, pose)
            error = [e[0]]
            self.last_target = target
            self.move_base = True
            # self.publishGoal(target)
        return error

    def update(self, enabled, key='', vmax=1.0, angle_rate = 5.0, vx=0.0, vy=0.0, w=0.0, jvx=0.0, jvy=0.0, jw=0.0) :
        self.enabled = enabled
        self.msg.Vx = vx
        self.msg.Vy = vy
        self.msg.w = w
        if key == 'joy' :
            self.msg.Vx = jvx
            self.msg.Vy = jvy
            self.msg.w = jw
        if self.key != key :
            self.last_target = None
            self.last_error = (0.0, 0.0)
        if key == 'move_base' :
            self.goal_pub = rospy.Publisher('/move_base_simple/goal',geomsg.PoseStamped,queue_size=1)
        self.key = key
        self.vmax = vmax
        self.angle_rate = angle_rate