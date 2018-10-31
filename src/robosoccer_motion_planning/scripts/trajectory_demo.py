#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import unicode_literals

import sys
import rospy
import rrtstar.msg as msg
import rrtstar.srv as srv

import sys
import os
import random
import matplotlib
# Make sure that we are using QT5
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtWidgets, QtGui

from math import ceil, floor
from numpy import arange, sin, pi, linspace, array
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class MyMplCanvas(FigureCanvas):
  """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""
  def __init__(self, parent=None, width=5, height=4, dpi=100):
    self.fig = Figure(figsize=(width, height), dpi=dpi)
    self.axes = self.fig.add_subplot(111)
    self.compute_initial_figure()
    FigureCanvas.__init__(self, self.fig)
    self.setParent(parent)
    FigureCanvas.setSizePolicy(self,
                               QtWidgets.QSizePolicy.Expanding,
                               QtWidgets.QSizePolicy.Expanding)
    FigureCanvas.updateGeometry(self)
  def compute_initial_figure(self):
    pass

class TrajectoryCanvas(MyMplCanvas) :
  def __init__(self, *args, **kwargs) :
    MyMplCanvas.__init__(self, *args, **kwargs)

  def update_figure(self, states, input=[]) :
    if len(states) < 1 : return
    t, x, u = [], [], []
    ymin, ymax, xmin, xmax = 5, -5, 5, -5
    for _ in range(states[0].n) :
      x.append([])
    for s in states :
      t.append(s.t.data.to_sec())
      xmin = s.t.data.to_sec() if xmin > s.t.data.to_sec() else xmin
      xmax = s.t.data.to_sec() if xmax < s.t.data.to_sec() else xmax
      for i in range(s.n) :
        x[i].append(s.state[i])
        # for nice plot
        ymin = s.state[i] if ymin > s.state[i] else ymin
        ymax = s.state[i] if ymax < s.state[i] else ymax
    if len(input):
      for _ in range(input[0].n) :
        u.append([])
      for a in input :
        for i in range(a.n) :
          u[i].append(a.state[i])
          # for nice plot
          ymin = a.state[i] if ymin > a.state[i] else ymin
          ymax = a.state[i] if ymax < a.state[i] else ymax
    self.axes.cla()
    ymin = ymin - (0.1 if ymin < 1.0 else 0.5)
    ymax = ymax + (0.1 if ymax < 1.0 else 0.5)
    for i in range(len(x)) :
      s = x[i]
      self.axes.plot(t, s, label='x%s'%i, alpha=0.95)
    for i in range(len(u)) :
      s = u[i]
      self.axes.plot(t, s, label='u%s'%i, dashes=[3,2], alpha=0.95)
    for y in arange(floor(ymin), ceil(ymax), 0.5):
      self.axes.plot(arange(xmin, xmax, 0.05), [y] * len(arange(xmin, xmax, 0.05)), dashes=[2,3], lw=0.5, color="black", alpha=0.3)
    self.axes.legend(loc='lower right', fancybox=True, framealpha=0.5)
    y_lim = self.axes.get_ylim()
    y_lim = [y + (-1 if y < 0 else 1) * (0.2 if abs(y) < 1.0 else 0.5) for y in y_lim]
    self.axes.set_ylim(y_lim)
    self.draw()

class ApplicationWindow(QtWidgets.QMainWindow):
  def __init__(self):
    QtWidgets.QMainWindow.__init__(self)
    self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
    self.setWindowTitle("application main window")
    self.file_menu = QtWidgets.QMenu('&File', self)
    self.file_menu.addAction('&Quit', self.fileQuit,
                             QtCore.Qt.CTRL + QtCore.Qt.Key_Q)
    self.menuBar().addMenu(self.file_menu)
    self.help_menu = QtWidgets.QMenu('&Help', self)
    self.menuBar().addSeparator()
    self.menuBar().addMenu(self.help_menu)
    self.main_widget = QtWidgets.QWidget(self)
    self.main_widget.setFocus()
    # prepare layout
    l = QtWidgets.QVBoxLayout(self.main_widget)
    # add plotter
    self.canvas = TrajectoryCanvas(self.main_widget, width=5, height=4, dpi=100)
    l.addWidget(self.canvas)
    # add widgets
    sb = QtWidgets.QDoubleSpinBox
    self.sboxes = {
      'initial' : [sb(), sb()],
      'final' : [sb()],
      'param' : [sb(), sb()]
      }
    for k in self.sboxes.keys() :
      for s in self.sboxes[k] :
        s.setMaximumWidth(50)
        s.setSingleStep(0.1)
    ihl, fhl = QtWidgets.QHBoxLayout(), QtWidgets.QHBoxLayout()
    ihl.addWidget(QtWidgets.QLabel('initial states :'))
    fhl.addWidget(QtWidgets.QLabel('final states :'))
    for s in self.sboxes['initial'] :
      s.setMinimum(-s.maximum())
      ihl.addWidget(s)
    for s in self.sboxes['final'] :
      s.setMinimum(-s.maximum())
      fhl.addWidget(s)
    # add btn to hlayout
    self.btn = {
      'apply' : QtWidgets.QPushButton('apply'),
      'save' : QtWidgets.QPushButton('save'),
    }
    for k, v in self.btn.iteritems() :
      v.setMaximumWidth(50)
    # hbox for params and btn
    bhl = QtWidgets.QHBoxLayout()
    label = QtWidgets.QLabel('limit :')
    label.setMaximumWidth(50)
    bhl.addWidget(label)
    for box in self.sboxes['param'] :
      bhl.addWidget(box)
      box.setMinimum(0.1)
    bhl.addWidget(QtWidgets.QWidget())
    for k, v in self.btn.iteritems() :
      bhl.addWidget(v)
    # add hlayout to main layout
    l.addLayout(ihl), l.addLayout(fhl), l.addLayout(bhl)
    self.setCentralWidget(self.main_widget)
    self.centralWidget().setContentsMargins(0.,0.,0.,0.)
    self.statusBar().showMessage("All hail matplotlib!", 2000)
    # connect btn to apply method
    self.btn['apply'].clicked.connect(self.apply)
    self.btn['save'].clicked.connect(self.save)
    rospy.Timer(rospy.Duration(2), self.check)

  def save(self) :
    filename = QtWidgets.QFileDialog.getSaveFileName()[0]
    self.canvas.fig.savefig(filename,bbox_inches="tight")

  def apply(self) :
    rospy.wait_for_service('compute_trajectory')
    trajectory = rospy.ServiceProxy('compute_trajectory', srv.TrajectoryService)
    try:
      req = srv.TrajectoryServiceRequest()
      req.model = 'angular_trajectory'
      req.params = [s.value() for s in self.sboxes['param']]
      req.xi.state = [s.value() for s in self.sboxes['initial']]
      req.xf.state = [s.value() for s in self.sboxes['final']]
      req.xi.n, req.xf.n = len(self.sboxes['initial']), len(self.sboxes['final'])
      res = trajectory.call(req)
      rospy.logwarn('service call success : %s'%res)
      self.canvas.update_figure(res.trajectory, res.inputs)
    except rospy.ServiceException as exc:
      rospy.logwarn('service call failed : %s'%exc)

  def fileQuit(self):
    self.close()

  def closeEvent(self, ce):
    self.fileQuit()

  def check(self, ev) :
    if rospy.is_shutdown() :
      self.close()

if __name__ == '__main__' :
  qApp = QtWidgets.QApplication(sys.argv)
  try: # join the dark side!
      import qdarkstyle
      qApp.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
  except ImportError:
      pass
  progname = 'angular_trajectory_demo'
  rospy.init_node(progname, sys.argv)
  rospy.logwarn('ready')
  aw = ApplicationWindow()
  aw.setWindowTitle("%s" % progname)
  aw.show()
  sys.exit(qApp.exec_())
