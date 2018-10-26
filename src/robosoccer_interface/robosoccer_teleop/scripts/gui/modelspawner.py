#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import gazebo_msgs.srv as gazebo_srv
import geometry_msgs

from PyQt5 import QtWidgets, QtGui, QtCore

class RobosoccerSpawner(object) :
    def __init__(self, *args, **kwargs) :
        self.spawner = rospy.ServiceProxy('/gazebo/spawn_sdf_model', gazebo_srv.SpawnModel)
        # self.prefix = {
            # 'rival' : rospy.get_param('/magenta_prefix', default='rival'),
            # 'nubot' : rospy.get_param('/cyan_prefix', default='nubot')
        # }
        pass
    
    def spawn(self, model_name, pos) :
        req = gazebo_srv.SpawnModelRequest()
        req.model_name = model_name
        rospack = rospkg.RosPack()
        file = rospack.get_path('nubot_description') + '/models/' + model_name + '/model.sdf'
        with open(file, 'r') as f:
          req.model_xml = f.read()
        req.initial_pose.position.x = pos[0]
        req.initial_pose.position.y = pos[1]
        res = self.spawner(req)
        rospy.logerr('model_xml %s' % req.model_xml)
        rospy.logerr('%s, %s' %(res.success, res.status_message))

class SpawnerDialog(QtWidgets.QDialog) :
    def __init__(self, parent=None, *args, **kwargs):
        QtWidgets.QDialog.__init__(self, parent)
        self.spawner = RobosoccerSpawner()
        l = QtWidgets.QGridLayout()
        self.widget = {
            'model' : QtWidgets.QLineEdit(self),
            'x' : QtWidgets.QDoubleSpinBox(self),
            'y' : QtWidgets.QDoubleSpinBox(self),
            'spawn' : QtWidgets.QPushButton('spawn',self),
            'cancel' : QtWidgets.QPushButton('cancel',self)
        }
        l.addWidget(QtWidgets.QLabel('model name :',self),0,0)
        l.addWidget(self.widget['model'],0,1)
        l.addWidget(QtWidgets.QLabel('x',self),1,0)
        l.addWidget(self.widget['x'],1,1)
        l.addWidget(QtWidgets.QLabel('y',self),2,0)
        l.addWidget(self.widget['y'],2,1)
        l.addWidget(self.widget['spawn'],3,0)
        l.addWidget(self.widget['cancel'],3,1)
        self.setLayout(l)
        self.widget['spawn'].clicked.connect(self.btn_ok)
        self.widget['cancel'].clicked.connect(self.btn_cancel)
        self.accepted.connect(self.spawn)

    def btn_ok(self) :
        self.accept()
    
    def btn_cancel(self) :
        self.reject()

    def spawn(self) :
        w = self.widget
        pos = [w['x'].value(), w['y'].value()]
        self.spawner.spawn(w['model'].text(), pos)
        pass
