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
        self.last_error = (0.0, 0.0)
    
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
                self.last_error = (0.0, 0.0)
                self.last_target = target
        angle =  math.radians(pose[2])
        c = math.cos(angle)
        s = math.sin(angle)
        dx = target[0] - pose[0]
        dy = target[1] - pose[1]
        dw = target[2] - pose[2]
        if math.fabs(dw) > 180.0 :
            dw -= math.copysign(360.0, -dw)
        MAX_VTRANS = 200.0
        MAX_VROT = 1.0
        KP_TRANS = 1.0
        KD_TRANS = 2.0
        KP_ROT = 0.5
        KD_ROT = 1.0
        lx = c*dx + s*dy
        ly = c*dy - s*dx
        error = math.hypot(lx, ly)
        lw = -math.atan2(ly, lx)
        lc = math.cos(lw)
        ls = math.sin(lw)
        d_error = self.last_error[0] - error
        d_error_rot = self.last_error[1] - dw
        V = KP_TRANS * error + KD_TRANS * d_error
        w = KP_ROT * dw + KD_ROT * d_error_rot
        if V > MAX_VTRANS :
            V = MAX_VTRANS
        self.msg.Vx = lc * V
        self.msg.Vy = ls * V
        self.msg.w = w
        self.last_error = (error, dw)
        return self.last_error

    def update(self, enabled, key='', vmax=1.0, angle_rate = 5.0, vx=0.0, vy=0.0, w=0.0) :
        self.enabled = enabled
        self.msg.Vx = vx
        self.msg.Vy = vy
        self.msg.w = w
        if self.key != key :
            self.last_target = None
            self.last_error = (0.0, 0.0)
        self.key = key
        self.vmax = vmax
        self.angle_rate = angle_rate