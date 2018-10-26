#!/usr/bin/python
from __future__ import unicode_literals
import sys
import os
import random
import matplotlib
# Make sure that we are using QT5
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtWidgets

from numpy import arange, sin, pi
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


class MyStaticMplCanvas(MyMplCanvas):
    """Simple canvas with a sine plot."""

    def compute_initial_figure(self):
        t = arange(0.0, 3.0, 0.01)
        s = sin(2*pi*t)
        self.axes.plot(t, s)

class MyDynamicMplCanvas(MyMplCanvas):
    """A canvas that updates itself every second with a new plot."""

    def __init__(self, *args, **kwargs):
        MyMplCanvas.__init__(self, *args, **kwargs)
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.update_figure)
        timer.start(1000)

    def compute_initial_figure(self):
        self.axes.plot([0, 1, 2, 3], [1, 2, 0, 4], 'r')

    def update_figure(self):
        # Build a list of 4 random integers between 0 and 10 (both inclusive)
        l = [random.randint(0, 10) for i in range(4)]
        self.axes.cla()
        self.axes.plot([0, 1, 2, 3], l, 'r')
        self.draw()

class TrajectoryCanvas(MyMplCanvas) :
    def __init__(self, *args, **kwargs) :
        if 'topic' in kwargs :
            self.topic = kwargs['topic']
            kwargs.pop('topic')
        else :
            self.topic = 'robosoccer_trajectory_pos'
        MyMplCanvas.__init__(self, *args, **kwargs)
        self.subscriber = rospy.Subscriber(self.topic, navmsg.Path, callback=self.update_figure, queue_size=3)
        self.time = rospy.get_rostime()

    def update_figure(self, msg) :
        rospy.loginfo('receive message')
        now = rospy.get_rostime()
        if (now - self.time).to_sec() > 0.25 :
          self.time = now
          t, x, y, w = [], [], [], []
          for pt in msg.poses :
              t.append(pt.header.stamp.to_sec())
              x.append(pt.pose.position.x)
              y.append(pt.pose.position.y)
              quaternion = (
                  pt.pose.orientation.x,
                  pt.pose.orientation.y,
                  pt.pose.orientation.z,
                  pt.pose.orientation.w)
              euler = tf.transformations.euler_from_quaternion(quaternion)
              yaw = euler[2]
              w.append(yaw)
          self.axes.cla()
          self.axes.plot(t, x)
          self.axes.plot(t, y)
          self.axes.plot(t, w)
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

        l = QtWidgets.QVBoxLayout(self.main_widget)
        pc = TrajectoryCanvas(self.main_widget, width=5, height=4, dpi=100, topic='robosoccer_trajectory_pos')
        vc = TrajectoryCanvas(self.main_widget, width=5, height=4, dpi=100, topic='robosoccer_trajectory_vel')
        l.addWidget(pc)
        l.addWidget(vc)

        self.main_widget.setFocus()
        self.setCentralWidget(self.main_widget)

        self.statusBar().showMessage("All hail matplotlib!", 2000)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.check)
        self.timer.start(250)

    def fileQuit(self):
        self.close()

    def closeEvent(self, ce):
        self.fileQuit()

    def check(self) :
        if rospy.is_shutdown() :
            self.close()

if __name__ == '__main__' :
    qApp = QtWidgets.QApplication(sys.argv)

    rospy.init_node('robosoccer_trajectory_vis', argv=sys.argv)

    aw = ApplicationWindow()
    aw.setWindowTitle("%s" % progname)
    aw.show()
    sys.exit(qApp.exec_())
    #qApp.exec_()
