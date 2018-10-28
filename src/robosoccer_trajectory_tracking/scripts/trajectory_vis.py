#!/usr/bin/python
from __future__ import unicode_literals
import sys
import os
import random
import matplotlib
# Make sure that we are using QT5
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtWidgets, QtGui
import pyqtgraph

from numpy import arange, sin, pi, linspace, array
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# ros stuff
import tf
import rospy
from nav_msgs import msg as navmsg
import std_msgs
import control_msgs
import geometry_msgs

progname = os.path.basename(sys.argv[0])
progversion = "0.1"

import nubot_common.msg as msg

class RobotSubscriber(object) :
  def __init__(self, topic, id, *args, **kwargs):
    self.sub = rospy.Subscriber(topic,msg.OminiVisionInfo,callback=self.callback)
    self.id = id
    self.pos, self.vel = None, None
    self.header = None

  def callback(self, info) :
    rospy.loginfo('receive message')
    for robot in info.robotinfo :
      self.header = info.header
      if robot.AgentID == self.id :
        self.pos = (robot.pos.x/100.0, robot.pos.y/100.0, robot.heading.theta)
        self.vel = (robot.vtrans.x/100.0, robot.vtrans.y/100.0, robot.vrot)
        break

class TrajectorySubscriber(object) :
  def __init__(self, topic, *args, **kwargs) :
    self.subscriber = rospy.Subscriber(topic, navmsg.Path, callback=self.callback, queue_size=3)
    self.t, self.x, self.y, self.w = [], [], [], []

  def callback(self, msg) :
    rospy.loginfo('receive message')
    t, x, y, w = [], [], [], []
    for i in range(len(msg.poses)) :
      pt = msg.poses[i]
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

class MyMplCanvas(FigureCanvas):
    """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)

        self.compute_initial_figure()

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                                   QtWidgets.QSizePolicy.Expanding,
                                   QtWidgets.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

    def compute_initial_figure(self):
        pass

class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.setWindowTitle("application main window")

        # menu bar settings
        self.file_menu = QtWidgets.QMenu('&File', self)
        self.file_menu.addAction('&Quit', self.fileQuit,
                                 QtCore.Qt.CTRL + QtCore.Qt.Key_Q)
        self.menuBar().addMenu(self.file_menu)
        self.menuBar().addSeparator()

        self.main_widget = QtWidgets.QWidget(self)

        pyqtgraph.setConfigOptions(antialias=True)

        # set max data to be visualized
        self.max_data = 100
        
        # ROS Subscriber :
        ### create robot subscriber
        agent_id = 1
        all_names = rospy.get_param_names()
        if '/agent_id' in all_names :
          agent_id = rospy.get_param('/agent_id')
        self.robot_sub = RobotSubscriber('/nubot'+str(agent_id)+'/omnivision/OmniVisionInfo', agent_id)
        ### create nav path subscriber
        self.nav_pos_sub = TrajectorySubscriber('robosoccer_trajectory_pos')
        self.nav_vel_sub = TrajectorySubscriber('robosoccer_trajectory_vel')

        # create plot widget
        self.plot_widget = {
          'pos' : pyqtgraph.PlotWidget(parent=self.main_widget),
          'vel' : pyqtgraph.PlotWidget(parent=self.main_widget)
        }
        # create plot item
        self.plot = {
          'pos' : {
            'x' : pyqtgraph.PlotDataItem(),
            'y' : pyqtgraph.PlotDataItem(),
            'w' : pyqtgraph.PlotDataItem()
          },
          'vel' : {
            'x' : pyqtgraph.PlotDataItem(),
            'y' : pyqtgraph.PlotDataItem(),
            'w' : pyqtgraph.PlotDataItem()
          },
        }
        self.plan_plot = {
          'pos' : {
            'x' : pyqtgraph.PlotDataItem(),
            'y' : pyqtgraph.PlotDataItem(),
            'w' : pyqtgraph.PlotDataItem()
          },
          'vel' : {
            'x' : pyqtgraph.PlotDataItem(),
            'y' : pyqtgraph.PlotDataItem(),
            'w' : pyqtgraph.PlotDataItem()
          },
        }
        # insert plot item to widget
        for k in self.plot.keys() :
          for kk in self.plot[k] :
            self.plot_widget[k].addItem(self.plot[k][kk])
        for k in self.plan_plot.keys() :
          for kk in self.plan_plot[k] :
            self.plot_widget[k].addItem(self.plan_plot[k][kk])

        # create place holder
        self.robot_data = {
          'pos' : { 'x' : [], 'y' : [], 'w' : [] },
          'vel' : { 'x' : [], 'y' : [], 'w' : [] },
          'time' : []
        }
        self.plan_data = {
          'pos' : { 'x' : [], 'y' : [], 'w' : [], 't' : [] },
          'vel' : { 'x' : [], 'y' : [], 'w' : [], 't' : [] },
        }
        
        # create layout and add plot widget to it
        l = QtWidgets.QVBoxLayout(self.main_widget)
        s = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        for k in self.plot_widget :
          s.addWidget(self.plot_widget[k])
        l.addWidget(s)

        self.main_widget.setFocus()
        self.setCentralWidget(self.main_widget)
        self.centralWidget().layout().setContentsMargins(0,0,0,0)

        self.setMinimumSize(400,400)

        self.statusBar().showMessage("All hail pyqtgraph!", 2000)

        # update gui using timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)
        self.gui_rate = 3
        self.gui_t0 = rospy.get_time()

    def fileQuit(self):
        self.close()

    def closeEvent(self, ce):
        self.fileQuit()

    def update(self) :
      self.update_plot()
      # check if rosmaster is ok
      if rospy.is_shutdown() :
          self.close()

    def update_plot(self) :
      # read time for gui
      now = rospy.get_time()
      # get subscriber, data, etc
      rpos, rvel = self.robot_sub.pos, self.robot_sub.vel
      npsub, nvsub = self.nav_pos_sub, self.nav_vel_sub
      time, pos, vel = self.robot_data['time'], self.robot_data['pos'], self.robot_data['vel']
      ppos, pvel = self.plan_data['pos'], self.plan_data['vel']
      # only plot if the message already published
      if (rpos is None) or (len(npsub.t) == 0) or (len(nvsub.t) == 0):
        return
      # get plot data item to be filled
      pplot, vplot = self.plot['pos'], self.plot['vel']
      pplotp, vplotp = self.plan_plot['pos'], self.plan_plot['vel']
      # let's save current planned time and state to our data
      idx = 0
      # first we find nearest time point
      # for i in range(len(npsub.t)) :
      #   dtime = npsub.t[i] - now
      #   if dtime > 0.0 :
      #     idx = i
      #     break
      ppos['t'].append(npsub.t[idx]), pvel['t'].append(nvsub.t[idx])
      ppos['x'].append(npsub.x[idx]), ppos['y'].append(npsub.y[idx]), ppos['w'].append(npsub.w[idx])
      pvel['x'].append(nvsub.x[idx]), pvel['y'].append(nvsub.y[idx]), pvel['w'].append(nvsub.w[idx])
      # add current robot position and time
      pos['x'].append(rpos[0]), pos['y'].append(rpos[1]), pos['w'].append(rpos[2])
      vel['x'].append(rvel[0]), vel['y'].append(rvel[1]), vel['w'].append(rvel[2])
      time.append(self.robot_sub.header.stamp.to_sec())
      # helper function
      def check_list(d, key, n) :
        for k in key :
          l = len(d[k])
          if len(d[k]) > n:
            del d[k][:l-n]
      # check if we should update gui
      if (now - self.gui_t0) > (1./self.gui_rate) :
        self.gui_t0 = now
        # only plot last n data
        w = self.max_data
        # delete first (len-w) data
        check_list(ppos, ppos.keys(), w)
        check_list(pvel, ppos.keys(), w)
        check_list(pos, pos.keys(), w)
        check_list(pos, vel.keys(), w)
        if(len(time) > w) : del time[:len(time)-w]
        # set data for planned trajectory which is combinatio of planned trajectory
        # so we could track robot position with planned trajectory
        plt_ppos = {
          'x': ppos['x']+npsub.x, 'y': ppos['y']+npsub.y, 
          'w': ppos['w']+npsub.w, 't': ppos['t']+npsub.t
          }
        plt_pvel = {
          'x': pvel['x']+nvsub.x, 'y': pvel['y']+nvsub.y, 
          'w': pvel['w']+nvsub.w, 't': pvel['t']+nvsub.t
          }
        # w = 0 if len(time) < w else w
        for k, j in [('x','red'), ('y','green'), ('w','blue')] :
          # prepare pen
          pen = QtGui.QPen(QtGui.QColor(j))
          pen.setWidthF(0.05)
          # set data for planed trajectory and actual robot pos
          pplot[k].setData(array(time[-w:]), array(pos[k][-w:]))
          vplot[k].setData(array(time[-w:]), array(vel[k][-w:]))
          pplotp[k].setData(array(plt_ppos['t'][-w:]), array(plt_ppos[k][-w:]))
          vplotp[k].setData(array(plt_pvel['t'][-w:]), array(plt_pvel[k][-w:]))
          # set pen for plotter
          pplot[k].setPen(pen), vplot[k].setPen(pen)
          pen.setStyle(QtCore.Qt.DotLine)
          pplotp[k].setPen(pen), vplotp[k].setPen(pen)
        # self.plot_widget['pos'].setXRange(plt_pvel['t'][-w],plt_pvel['t'][-1])
        # self.plot_widget['vel'].setXRange(plt_pvel['t'][-w],plt_pvel['t'][-1])
        # self.plot_widget['vel'].setYRange(1.75,-1.75)

if __name__ == '__main__' :
    qApp = QtWidgets.QApplication(sys.argv)

    rospy.init_node('robosoccer_trajectory_vis', argv=sys.argv)

    aw = ApplicationWindow()
    aw.setWindowTitle("%s" % progname)
    aw.show()
    sys.exit(qApp.exec_())
    #qApp.exec_()
