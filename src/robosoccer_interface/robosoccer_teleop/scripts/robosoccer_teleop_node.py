#!/usr/bin/python
import gui.teleopgui as gui
import rospy
import sys

from PyQt5 import QtWidgets, QtGui, QtCore

if __name__ == '__main__' :
    app = QtWidgets.QApplication(sys.argv)
    teleop = gui.TeleopGUI()
    rospy.init_node('robosoccer_teleop')
    teleop.widget.showMaximized()
    app.exec_()
    sys.exit(0)