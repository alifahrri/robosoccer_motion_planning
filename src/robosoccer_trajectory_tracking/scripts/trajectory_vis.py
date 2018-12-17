#!/usr/bin/python
from __future__ import unicode_literals
import os
import sys
import copy
import random
import matplotlib
# Make sure that we are using QT5
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtWidgets, QtGui
import pyqtgraph
import pyqtgraph.exporters

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
import rosgraph_msgs.msg as rosmsg
import robosoccer_trajectory_tracking.msg as trackmsg

# for measuring rate
import rostopic

class ControlInfoSubscriber() :
  def __init__(self, topic) :
    self.sub = rospy.Subscriber(topic,trackmsg.ControlInfo,callback=self.callback,queue_size=3)
    self.clock_sub = rospy.Subscriber('/clock',rosmsg.Clock,callback=self.clock,queue_size=1)
    self.time = None
    self.error = {
      'p' : {
        't' : [], 'x' : [],
        'y' : [], 'w' : []
      },
      'i' : {
        't' : [], 'x' : [],
        'y' : [], 'w' : []
      }
    }
  
  def clock(self, msg) :
    # msg = rosmsg.Clock
    self.time = msg.clock

  def get_error(self) :
    return self.error

  def callback(self, msg) :
    # msg = trackmsg.ControlInfo()
    if self.time is None :
      return
    keys = [('p','x'),('p','y'),('p','w'),('i','x'),('i','y'),('i','w')]
    # t = rospy.get_rostime().to_time()
    t = self.time.to_time()
    for i in range(len(keys)) :
      k = keys[i]
      self.error[k[0]][k[1]].append(msg.info[i])
    for k in ['p', 'i'] :
      self.error[k]['t'].append(t)

class RobotSubscriber(object) :
  def __init__(self, topic, id, *args, **kwargs):
    self.sub = rospy.Subscriber(topic,msg.OminiVisionInfo,callback=self.callback,queue_size=3)
    self.id = id
    self.pos, self.vel = None, None
    self.header = None
    self.states = {
      'pos' : {
        't' : [], 'x' : [],
        'y' : [], 'w' : []
      },
      'vel' : {
        't' : [], 'x' : [],
        'y' : [], 'w' : []
      }
    }

  def get_states(self) :
    return self.states

  def callback(self, info) :
    rospy.loginfo('receive robot states message')
    self.header = info.header
    for robot in info.robotinfo :
      if robot.AgentID == self.id :
        self.pos = (robot.pos.x/100.0, robot.pos.y/100.0, robot.heading.theta)
        self.vel = (robot.vtrans.x/100.0, robot.vtrans.y/100.0, robot.vrot)
        keys = ['x', 'y', 'w']
        self.states['pos']['t'].append(info.header.stamp.to_sec())
        self.states['vel']['t'].append(info.header.stamp.to_sec())
        for k in range(len(keys)) :
          self.states['pos'][keys[k]].append(self.pos[k])
          self.states['vel'][keys[k]].append(self.vel[k])
        return

class TrajectorySubscriber(object) :
  def __init__(self, topic, *args, **kwargs) :
    self.subscriber = rospy.Subscriber(topic, navmsg.Path, callback=self.callback, queue_size=3)
    self.t, self.x, self.y, self.w = [], [], [], []

  def callback(self, msg) :
    rospy.loginfo('receive trajectory message')
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
      # if not all([c is not None for c in [t[-1], x[-1], y[-1], w[-1]]]) :
      #   return
    if not (len(t) == len(x) == len(y) == len(w)) :
      return
    m = 0
    for i in range(len(self.t)) :
      if i == 0 : continue
      m = -i
      if self.t[m] <= t[0] :
        self.t = self.t[:m]
        self.x = self.x[:m]
        self.y = self.y[:m]
        self.w = self.w[:m]
        break
    self.t = self.t + t 
    self.x = self.x + x 
    self.y = self.y + y 
    self.w = self.w + w
    # self.t = self.t[:m] + t 
    # self.x = self.x[:m] + x 
    # self.y = self.y[:m] + y 
    # self.w = self.w[:m] + w

  def get_plan(self) :
    return self.t, self.x, self.y, self.w

class MyMplCanvas(FigureCanvas):
    """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""

    def __init__(self, parent=None, width=5, height=4, dpi=100, title=''):
        # fig = Figure(figsize=(width, height), dpi=dpi)
        self.fig = Figure(dpi=dpi)
        self.title = title
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

  def update_figure(self, t, x, tdash=[], xdash=[], label=(['x','y','w'],['x','y','w']), trange=None, legend=False) :
    self.axes.cla()
    for i in range(len(x)) :
      s = x[i]
      l = ''
      try : l = label[0][i]
      except (IndexError,TypeError) : pass
      self.axes.plot(t, s, label=l, alpha=0.95)
    for i in range(len(xdash)) :
      s = xdash[i]
      l = ''
      try : l = label[1][i]
      except (IndexError,TypeError) : pass
      self.axes.plot(tdash, s, label=l, dashes=[3,2], alpha=0.95)
    if legend :
      self.axes.legend(loc='upper center', bbox_to_anchor=(0.5, 0.995), ncol=3, fancybox=True, framealpha=0.5)
      # self.axes.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)
      # self.axes.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', ncol=2, mode="expand", borderaxespad=0.)
      # self.axes.legend(loc='upper right', fancybox=True, framealpha=0.5)
    if not (trange is None) :
      self.axes.autoscale_view()
      xlim = self.axes.get_xlim()
      if trange[0] < 0 :
        xlim = (xlim[1] + trange[0], xlim[1])
      else :
        xlim = (trange[0], trange[1] if trange[1] > trange[0] else xlim[1])
      self.axes.set_xlim(xlim)
    self.axes.set_title(self.title)
    self.axes.grid()
    self.draw()

class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.setWindowTitle("application main window")

        # menu bar settings
        self.file_menu = QtWidgets.QMenu('&File', self)
        self.file_menu.addAction('&Save', self.save,
                                 QtCore.Qt.CTRL + QtCore.Qt.Key_S)
        self.file_menu.addAction('&Quit', self.fileQuit,
                                 QtCore.Qt.CTRL + QtCore.Qt.Key_Q)
        self.menuBar().addMenu(self.file_menu)
        self.menuBar().addSeparator()

        self.main_widget = QtWidgets.QWidget(self)

        pyqtgraph.setConfigOptions(antialias=True)

        # set max data to be visualized
        self.max_data = int(1e7)
        
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
        ### create trajectory tracker info subscriber (for error)
        self.trackinfo_sub = ControlInfoSubscriber('/tracker_info')

        tc = TrajectoryCanvas

        # create plot widget
        ## self.plot_widget = {
        ##   'pos' : pyqtgraph.PlotWidget(parent=self.main_widget),
        ##   'vel' : pyqtgraph.PlotWidget(parent=self.main_widget)
        ## }

        # error plotter dialog
        self.err_dialog = QtWidgets.QDialog(self.main_widget)

        # plot widget (matplotlib)
        self.plot_widget = {
          'pos' : tc(parent=self.main_widget, width=5, height=4, dpi=50, title='Robot Pose'),
          'vel' : tc(parent=self.main_widget, width=5, height=4, dpi=50, title='Body Rate'),
          'err' : tc(parent=self.main_widget, width=5, height=4, dpi=50, title='Error')
        }

        # create place holder
        self.robot_data = {
          'pos' : { 'x' : [], 'y' : [], 'w' : [], 't' : [] },
          'vel' : { 'x' : [], 'y' : [], 'w' : [], 't' : [] },
        }
        self.plan_data = {
          'pos' : { 'x' : [], 'y' : [], 'w' : [], 't' : [] },
          'vel' : { 'x' : [], 'y' : [], 'w' : [], 't' : [] },
        }
        self.error_data = {
          'p' : { 't' : [], 'x' : [], 'y' : [], 'w' : [] },
          'i' : { 't' : [], 'x' : [], 'y' : [], 'w' : [] }
        }
        
        # gui controls
        self.ctrl = {
          'min' : QtWidgets.QDoubleSpinBox(),
          'max' : QtWidgets.QDoubleSpinBox(),
          'label' : QtWidgets.QCheckBox('label'),
          'err' : QtWidgets.QPushButton('show error plot')
        }

        # create layout and add plot widget to it
        l = QtWidgets.QVBoxLayout(self.main_widget)
        s = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        h = QtWidgets.QHBoxLayout(self.main_widget)
        h.addWidget(QtWidgets.QLabel('[min,max] : '))
        for k in ['min', 'max'] :
          self.ctrl[k].setMaximum(700.0)
          self.ctrl[k].setMinimum(-700.0)
          h.addWidget(self.ctrl[k])
        h.addWidget(self.ctrl['label'])
        # h.addWidget(self.ctrl['err'])
        l.addLayout(h)
        for k in self.plot_widget :
          l.addWidget(self.plot_widget[k])
        # l.addWidget(s)

        self.main_widget.setFocus()
        self.setCentralWidget(self.main_widget)
        self.centralWidget().layout().setContentsMargins(0,0,0,0)

        self.setMinimumSize(400,400)

        self.statusBar().showMessage("All hail matplotlib!", 2000)

        # update gui using timer
        # self.timer = QtCore.QTimer()
        # self.timer.timeout.connect(self.update)
        # self.timer.start(100)
        self.need_update = True
        self.timer = rospy.Timer(rospy.Duration(0,int(100e6)), self.update)
        self.gui_rate = 10.
        self.gui_t0 = rospy.get_time()
        self.ctrl['err'].clicked.connect(self.show_error_plot)

    def show_error_plot(self) :
      self.err_dialog.show()

    def save(self) :
      self.need_update = False
      for k in self.plot_widget.keys() :
        filename = QtWidgets.QFileDialog.getSaveFileName(self.main_widget,'save %s figure'%k,os.environ['HOME'],"Images (*.png)")[0]
        self.plot_widget[k].fig.savefig(filename,bbox_inches="tight")

    def fileQuit(self):
        self.close()

    def closeEvent(self, ce):
        self.fileQuit()

    def update(self, event) :
      # check if rosmaster is ok
      if rospy.is_shutdown() :
        self.close()
      if self.need_update :
        self.update_plot()

    def update_plot(self) :
      # read time for gui
      now = rospy.get_time()
      # get subscriber, data, etc
      npsub, nvsub = self.nav_pos_sub, self.nav_vel_sub
      # check if we should update gui
      if (now - self.gui_t0) > (1./self.gui_rate) :
        pplan = npsub.get_plan()
        vplan = nvsub.get_plan()
        plan_data = {
          'pos' : {
            't' : copy.copy(pplan[0]), 'x' : copy.copy(pplan[1]),
            'y' : copy.copy(pplan[2]), 'w' : copy.copy(pplan[3])
          },
          'vel' : {
            't' : copy.copy(vplan[0]), 'x' : copy.copy(vplan[1]),
            'y' : copy.copy(vplan[2]), 'w' : copy.copy(vplan[3])
          }
        }
        states = copy.copy(self.robot_sub.get_states())
        error_data = copy.copy(self.trackinfo_sub.get_error())
        # helper function
        def set_elements(d1, d2) :
          keys = ['t', 'x', 'y', 'w']
          length = min([len(d2[k]) for k in keys])
          for k in keys :
            d1[k] = d2[k][:length]
        def append(d1, d2) :
          keys = ['t', 'x', 'y', 'w']
          for k in keys :
            d1[k] = d1[k] + d2[k][len(d1[k]):]

        for k in ['pos', 'vel'] :
          set_elements(self.plan_data[k], plan_data[k])
          append(self.robot_data[k], states[k])
        for k in ['p', 'i'] :
          append(self.error_data[k], error_data[k])

        states = self.robot_data
        plan_data = self.plan_data
        error_data = self.error_data
        # pos, vel = states['pos'], states['vel']
        # if (not len(pos)) or (not len(vel)) : 
        #   return
        self.gui_t0 = now
        def mplplot(widget, data={'t':[],'x':[],'y':[],'w':[]}, dash={'t':[],'x':[],'y':[],'w':[]}, range=None, label=None) :
          x = [data['x'], data['y'], data['w']]
          xd = [dash['x'], dash['y'], dash['w']]
          legend = False if label is None else self.ctrl['label'].isChecked()
          widget.update_figure(data['t'],x,tdash=dash['t'],xdash=xd,trange=range,legend=legend,label=label)
        def check_length(dicts) :
          for data in dicts :
            x = [data['x'], data['y'], data['w']]
            if not (len(data['t']) == len(x[0]) == len(x[1]) == len(x[2])) :
                return False
          return True
        xlim = (self.ctrl['min'].value(), self.ctrl['max'].value())
        xlim = None if (xlim==(0,0)) else xlim
        if check_length([plan_data['pos'], plan_data['vel'], states['pos'], states['vel'], error_data['p'], error_data['i']]) :
          mplplot(self.plot_widget['pos'],data=states['pos'],dash=plan_data['pos'],range=xlim,label=(['x','y','w'],['x(plan)','y(plan)','w(plan)']))
          mplplot(self.plot_widget['vel'],data=states['vel'],dash=plan_data['vel'],range=xlim,label=(['vx(control)','vy(control)','vw(control)'],['vx(plan)','vy(plan)','vw(plan)']))
          mplplot(self.plot_widget['err'],data=error_data['p'],range=xlim,label=(['ex','ey','ew'],[]))


if __name__ == '__main__' :
    qApp = QtWidgets.QApplication(sys.argv)
    try:
        import qdarkstyle
        qApp.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    except ImportError:
        pass

    rospy.init_node('robosoccer_trajectory_vis', argv=sys.argv)

    aw = ApplicationWindow()
    aw.setWindowTitle("%s" % progname)
    aw.show()
    sys.exit(qApp.exec_())
    #qApp.exec_()
