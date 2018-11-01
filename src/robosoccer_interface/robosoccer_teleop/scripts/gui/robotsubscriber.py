from PyQt5 import QtWidgets, QtGui, QtCore
import rospy
import math
import copy
import sensor_msgs.msg as sensormsg
import nubot_common.msg as msg

class JoySubscriber() :
    def __init__(self, *args, **kwargs):
        topic = '/joystick'
        self.sub = rospy.Subscriber(topic,sensormsg.Joy,callback=self.callback)
        self.speed = {
            'trans' : {
                'x' : .0, 'y' : .0
            },
            'rot' : .0
        }
    
    def callback(self, msg) :
        axes = msg.axes
        btn = msg.buttons
        self.speed = {
            'trans' : {
                'x' : axes[1] * 100.0,
                'y' : axes[0] * 100.0
            },
            'rot' : btn[4] * .35 + btn[5] * (-.35) + btn[6] * .75 + btn[7] * (-.75)
        }

class RobotSubscriber(object) :
    def __init__(self, topic, id, *args, **kwargs):
        self.sub = rospy.Subscriber(topic,msg.OminiVisionInfo,callback=self.callback)
        self.id = id
        self.pos = None

    def callback(self, info) :
        for robot in info.robotinfo :
            if robot.AgentID == self.id :
                self.pos = (robot.pos.x, robot.pos.y, robot.heading.theta)
                break

class RobotItem(QtWidgets.QGraphicsItem) :
    def __init__(self, topic, id, flip=1, *args, **kwargs):
        QtWidgets.QGraphicsItem.__init__(self, None)
        self.topic_name = topic
        self.agent_id = id
        self.subscriber = RobotSubscriber(topic, id)
        self.flip = flip
        self.waypoints, self.orientation  = [], []
        self.wp_target = 0

    def setWaypoints(self, wp, o=[]) :
        del self.waypoints[:]
        del self.orientation[:]
        self.wp_target = 0
        if not (wp is None) :
            self.waypoints = list(wp)
            self.orientation = o + [0.0 for _ in range(len(wp)-len(o))]

    def nextTarget(self) :
        if self.waypoints is None : 
            return
        self.wp_target = (self.wp_target+1) % len(self.waypoints)

    def target(self) :
        ret = None
        if (not (self.waypoints is None)) and (len(self.waypoints) > 0) :
            wp = self.waypoints[self.wp_target]
            o = self.orientation[self.wp_target]
            ret = (wp.x(), wp.y(), o)
        return ret
    
    def pose(self) :
        ret = None
        if not (self.subscriber.pos is None) :
            pos = self.subscriber.pos
            w = pos[2]
            if self.flip < 0 :
                w = w + math.pi if w < 0 else w - math.pi
            ret = (self.flip*pos[0], -self.flip*pos[1], w)
        return ret

    def paint(self, painter, option, style) :
        if self.subscriber.pos is None :
            return
        transform = painter.transform()
        pos = self.subscriber.pos
        pos = (self.flip*pos[0], -self.flip*pos[1], pos[2])
        color = QtGui.QColor('blue')
        if self.flip < 0 :
            color = QtGui.QColor('red')
        robot_point = QtCore.QPointF(pos[0], pos[1])
        painter.setPen(color)
        painter.setBrush(color)
        ROBOT_SIZE = 60
        painter.drawEllipse(pos[0] - ROBOT_SIZE/2, pos[1] - ROBOT_SIZE/2, ROBOT_SIZE, ROBOT_SIZE)
        painter.translate(robot_point)
        painter.rotate(math.degrees(-pos[2])+180.0)
        p1 = QtCore.QPointF(0.0, 0.0)
        p2 = QtCore.QPointF(-self.flip*ROBOT_SIZE*1.25, 0.0)
        painter.drawLine(p1, p2)
        painter.setPen(QtGui.QColor('black'))
        font = painter.font()
        font.setPixelSize(24)
        text_rect = QtCore.QRectF(-ROBOT_SIZE,-ROBOT_SIZE,ROBOT_SIZE*2,ROBOT_SIZE*2)
        painter.setFont(font)
        # txt = str(self.agent_id) + ' : (%s,%s,%s)' % (pos[0], pos[1], pos[2])
        txt = str(self.agent_id)
        painter.rotate(-180.0)
        painter.drawText(text_rect, QtCore.Qt.AlignCenter, txt)
        painter.setTransform(transform)
        for i in range(len(self.waypoints)) :
            p1 = self.waypoints[i]
            painter.setPen(color)
            painter.setBrush(color)
            if i > 0 :
                p0 = self.waypoints[i-1]
                painter.drawLine(p0, p1)
            if i ==  self.wp_target :
                painter.setPen(QtCore.Qt.green)
                painter.setBrush(QtCore.Qt.green)
                pen = painter.pen()
                pen.setStyle(QtCore.Qt.DashDotDotLine)
                painter.setPen(pen)
                painter.drawLine(robot_point, p1)
            painter.drawEllipse(p1, 10.0, 10.0)
            if len(self.orientation) > i :
                p = QtCore.QPointF(p1)
                angle = self.orientation[i]
                p.setX(p.x() + 30.0 * math.cos(angle))
                p.setY(p.y() + 30.0 * math.sin(angle))
                painter.drawLine(p1,p)
            font = painter.font()
            font.setPixelSize(12)
            painter.setFont(font)
            painter.setPen(QtCore.Qt.green)
            painter.drawText(p1, str(i))

    def boundingRect(self) :
        return QtCore.QRectF(-500,-500,1000,1000)
