import rospy
import math
import copy
import nubot_common.msg as msg
from PyQt5 import QtWidgets, QtGui, QtCore

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
        self.waypoints = []
        self.wp_target = 0

    def setWaypoints(self, wp) :
        del self.waypoints[:]
        if not (wp is None) :
            self.waypoints = list(wp)

    def nextTarget(self) :
        if self.waypoints is None : 
            return
        self.wp_target = (self.wp_target+1) % len(self.waypoints)

    def target(self) :
        ret = None
        if (not (self.waypoints is None)) and (len(self.waypoints) > 0) :
            wp = self.waypoints[self.wp_target]
            ret = (wp.x(), wp.y(), 0.0)
        return ret
    
    def pose(self) :
        ret = None
        if not (self.subscriber.pos is None) :
            pos = self.subscriber.pos
            if self.flip < 0 :
                ret = (self.flip*pos[0], pos[1], pos[2]-math.radians(180.0))
            else :
                ret = (pos[0], -self.flip*pos[1], pos[2])
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
            font = painter.font()
            font.setPixelSize(12)
            painter.setFont(font)
            painter.setPen(QtCore.Qt.green)
            painter.drawText(p1, str(i))

    def boundingRect(self) :
        return QtCore.QRectF(-500,-500,1000,1000)