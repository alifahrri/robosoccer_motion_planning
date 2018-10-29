import rospy
import math
import nubot_common.msg as msg

class VelocityPublisher(object) :
    def __init__(self, topic, *args, **kwargs):
        self.publisher = rospy.Publisher(topic,msg.VelCmd,queue_size=1)
        self.msg = msg.VelCmd()
        self.enabled = False
        self.key = 'stop'
        self.angle = 0.0
        self.angle_rate = 5.0
        self.vmax = 1.0
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
        self.publisher.publish(vel)

    def pidControl(self, target, pose) :
        if (self.key != 'waypoint') or (target is None) or (pose is None) :
            return None
        if not (self.last_target is None) :
            if target != self.last_target :
                self.last_error = (None, None)
                self.last_target = target
        angle = pose[2]
        c, s = math.cos(angle), math.sin(angle)
        dx = target[0] - pose[0]
        # here, target given from gui has -y
        dy = target[1] - pose[1]
        dw = math.radians(target[2]) - pose[2]
        if math.fabs(dw) > math.pi :
            dw -= math.copysign(math.pi, -dw)
        gain = self.pd_gain
        MAX_VTRANS, MAX_VROT = 200.0, 1.0
        KP_TRANS, KD_TRANS = gain['trans']['p'], gain['trans']['d']
        KP_ROT, KD_ROT = gain['rot']['p'], gain['trans']['d']
        lx, ly = c*dx - s*dy, c*dy + s*dx
        error = math.hypot(lx, ly)
        lw = math.atan2(-ly, lx)
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
        self.key = key
        self.vmax = vmax
        self.angle_rate = angle_rate