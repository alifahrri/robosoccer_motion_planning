import rospy
import math
import nubot_common.msg as msg

class VelocityPublisher(object) :
    def __init__(self, topic, *args, **kwargs):
        self.publisher = rospy.Publisher(topic,msg.VelCmd,queue_size=1)
        self.msg = msg.VelCmd()
        self.enabled = False
        self.key = ''
        self.angle = 0.0
        self.angle_rate = 5.0
        self.vmax = 1.0
    
    def publish(self) :
        vel = self.msg
        if not self.enabled :
            return
        elif self.key == 'circular' :
            vm = self.vmax
            angle = math.radians(self.angle)
            vel.Vx = math.cos(angle)*vm
            vel.Vy = math.sin(angle)*vm
            vel.w = 0.0
            self.angle = self.angle + self.angle_rate
        self.publisher.publish(vel)

    def update(self, enabled, key='', vmax=1.0, angle_rate = 5.0, vx=0.0, vy=0.0, w=0.0) :
        self.enabled = enabled
        self.msg.Vx = vx
        self.msg.Vy = vy
        self.msg.w = w
        self.key = key
        self.vmax = vmax
        self.angle_rate = angle_rate