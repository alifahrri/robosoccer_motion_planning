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
  def __init__(self, parent=None, width=5, height=4, dpi=100, title='', xlabel='', ylabel=''):
    self.fig = Figure(figsize=(width, height), dpi=dpi)
    self.title = title
    self.xlabel = xlabel
    self.ylabel = ylabel
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

  def update_compare_figure(self, states, labels=[]) :
    if len(states) < 1 : return
    self.axes.cla()
    for i in range(len(states)) :
      label = '' if i >= len(labels) else labels[i]
      s = states[i]
      if s.n == 1:
        self.axes.plot(s.t, s.state, label=label, alpha=0.95)
    if len(labels) :
      self.axes.legend(loc='best', fancybox=True, framealpha=0.5, prop={'size': 6})
    self.axes.set_title(self.title)
    self.axes.set_xlabel(self.xlabel)
    self.axes.set_ylabel(self.ylabel)
    self.axes.grid()
    y_lim = self.axes.get_ylim()
    y_lim = [1.1 * (y_lim[0]-0.1), 1.1 * (y_lim[1]+0.1)]
    self.axes.set_ylim(y_lim)
    # x_lim = self.axes.get_xlim()
    self.draw()
    
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
    self.tab = QtWidgets.QTabWidget(self.main_widget)
    ## prepare tab page
    self.page = {
      'demo' : QtWidgets.QWidget(self.tab),
      'compare' : QtWidgets.QWidget(self.tab),
    }
    for k, v in self.page.iteritems() :
      self.tab.addTab(v,k)

    # prepare layout
    l = QtWidgets.QVBoxLayout(self.page['demo'])
    # add plotter
    self.canvas = TrajectoryCanvas(self.page['demo'], width=5, height=4, dpi=100)
    l.addWidget(self.canvas)
    self.create_demo_layout(l)
    self.page['demo'].setLayout(l)

    # create placeholder for compare parameter
    self.compare_list = {
      'param' : [],
      'init' : [],
      'final' : []
    }

    # prepare compare layout
    self.compare_canvas = {
      'position' : TrajectoryCanvas(self.page['compare'], width=5, height=4, dpi=100, ylabel='position', xlabel='time'),
      'velocity' : TrajectoryCanvas(self.page['compare'], width=5, height=4, dpi=100, ylabel='velocity', xlabel='time'),
      'acceleration' : TrajectoryCanvas(self.page['compare'], width=5, height=4, dpi=100, ylabel='acceleration', xlabel='time'),
    }
    g = QtWidgets.QGridLayout()
    cl = QtWidgets.QVBoxLayout(self.page['compare'])
    g.addWidget(self.compare_canvas['position'],0,0)
    g.addWidget(self.compare_canvas['velocity'],0,1)
    g.addWidget(self.compare_canvas['acceleration'],0,2)
    cl.addLayout(g)
    self.create_compare_layout(cl)
    self.page['compare'].setLayout(cl)
    
    self.setCentralWidget(self.main_widget)
    self.layout = QtWidgets.QVBoxLayout(self.main_widget)
    self.layout.addWidget(self.tab)
    self.centralWidget().setContentsMargins(0.,0.,0.,0.)
    self.statusBar().showMessage("All hail matplotlib!", 2000)
    # connect btn to apply method
    self.btn['apply'].clicked.connect(self.apply)
    self.btn['save'].clicked.connect(self.save)
    self.compare_btn['apply'].clicked.connect(self.compare)
    self.compare_btn['save'].clicked.connect(self.save_compare)
    self.compare_btn['add'].clicked.connect(self.add_compare_param)
    rospy.Timer(rospy.Duration(2), self.check)

  def create_demo_layout(self, l) :
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

  def create_compare_layout(self, l) :
    ## create table for params
    self.compare_table = QtWidgets.QTableWidget()
    self.compare_table.setColumnCount(3)
    # add widgets
    sb = QtWidgets.QDoubleSpinBox
    self.compare_sboxes = {
      'initial' : [sb(), sb()],
      'final' : [sb()],
      'param' : [sb(), sb()]
      }
    for k in self.compare_sboxes.keys() :
      for s in self.compare_sboxes[k] :
        s.setMaximumWidth(50)
        s.setSingleStep(0.1)
    ihl, fhl = QtWidgets.QHBoxLayout(), QtWidgets.QHBoxLayout()
    ihl.addWidget(QtWidgets.QLabel('initial states :'))
    fhl.addWidget(QtWidgets.QLabel('final states :'))
    for s in self.compare_sboxes['initial'] :
      s.setMinimum(-s.maximum())
      ihl.addWidget(s)
    for s in self.compare_sboxes['final'] :
      s.setMinimum(-s.maximum())
      fhl.addWidget(s)
    # add btn to hlayout
    self.compare_btn = {
      'apply' : QtWidgets.QPushButton('apply'),
      'save' : QtWidgets.QPushButton('save'),
      'add' : QtWidgets.QPushButton('add'),
    }
    for k, v in self.compare_btn.iteritems() :
      v.setMaximumWidth(50)
    # hbox for params and btn
    bhl = QtWidgets.QHBoxLayout()
    label = QtWidgets.QLabel('limit :')
    label.setMaximumWidth(50)
    bhl.addWidget(label)
    for box in self.compare_sboxes['param'] :
      bhl.addWidget(box)
      box.setMinimum(0.1)
    bhl.addWidget(QtWidgets.QWidget())
    for k, v in self.compare_btn.iteritems() :
      bhl.addWidget(v)
    # add hlayout to main layout
    l.addWidget(self.compare_table)
    l.addLayout(ihl), l.addLayout(fhl), l.addLayout(bhl)

  def draw_compare_table(self) :
    n = len(self.compare_list['param'])
    self.compare_table.clear()
    self.compare_table.setColumnCount(4)
    self.compare_table.setRowCount(n)
    item = QtWidgets.QTableWidgetItem
    cmp_list = self.compare_list
    for i in range(n) :
      param, init, final = cmp_list['param'][i], cmp_list['init'][i], cmp_list['final'][i]
      self.compare_table.setItem(i, 0, item(str(param)))
      self.compare_table.setItem(i, 1, item(str(init)))
      self.compare_table.setItem(i, 2, item(str(final)))

  def add_compare_param(self) :
    param = [s.value() for s in self.compare_sboxes['param']]
    initial = [s.value() for s in self.compare_sboxes['initial']]
    final = [s.value() for s in self.compare_sboxes['final']]
    self.compare_list['param'].append(param)
    self.compare_list['init'].append(initial)
    self.compare_list['final'].append(final)
    self.draw_compare_table()

  def save(self) :
    filename = QtWidgets.QFileDialog.getSaveFileName()[0]
    self.canvas.fig.savefig(filename,bbox_inches="tight")

  def save_compare(self) :
    for k in ['position', 'velocity', 'acceleration'] :
      filename = QtWidgets.QFileDialog.getSaveFileName()[0]
      self.compare_canvas[k].fig.savefig(filename,bbox_inches="tight")

  def compare(self) :
    label = []
    for i in range(self.compare_table.rowCount()) :
      item = self.compare_table.item(i,3)
      text = item.text() if not (item is None) else ''
      label.append(text)
    class compare_data() :
      def __init__(self) :
        self.n = 1
        self.state = []
        self.t = []
    cmp_pos, cmp_vel, cmp_acc = [], [], []
    rospy.wait_for_service('compute_trajectory')
    trajectory = rospy.ServiceProxy('compute_trajectory', srv.TrajectoryService)
    for i in range(len(self.compare_list['param'])) :
      param = self.compare_list['param'][i]
      final = self.compare_list['final'][i]
      init = self.compare_list['init'][i]
      try:
        req = srv.TrajectoryServiceRequest()
        req.model = 'angular_trajectory'
        req.params = param
        req.xi.state = init
        req.xf.state = final
        req.xi.n, req.xf.n = len(self.sboxes['initial']), len(self.sboxes['final'])
        res = trajectory.call(req)
        rospy.logwarn('service call success : %s'%res)
        pos, vel, acc = compare_data(), compare_data(), compare_data()
        t = [t.t.data.to_sec() for t in res.trajectory]
        pos.state = [x.state[0] for x in res.trajectory]
        vel.state = [x.state[1] for x in res.trajectory]
        acc.state = [x.state[0] for x in res.inputs]
        pos.t, vel.t, acc.t = t, t, t
        cmp_pos.append(pos)
        cmp_vel.append(vel)
        cmp_acc.append(acc)
        self.canvas.update_figure(res.trajectory, res.inputs)
      except rospy.ServiceException as exc:
        rospy.logwarn('service call failed : %s'%exc)
    self.compare_canvas['position'].update_compare_figure(cmp_pos, labels=label)
    self.compare_canvas['velocity'].update_compare_figure(cmp_vel,labels=label)
    self.compare_canvas['acceleration'].update_compare_figure(cmp_acc, labels=label)

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
