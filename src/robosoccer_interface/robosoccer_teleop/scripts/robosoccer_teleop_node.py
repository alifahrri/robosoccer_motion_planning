#!/usr/bin/python
import gui.teleopgui as gui
import rospy
import sys

import qdarkstyle
modulename = 'qdarkstyle'

from PyQt5 import QtWidgets, QtGui, QtCore

if __name__ == '__main__' :
    app = QtWidgets.QApplication(sys.argv)
    if modulename in sys.modules:
        app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    teleop = gui.TeleopGUI()
    rospy.init_node('robosoccer_teleop')
    teleop.showMaximized()
    app.exec_()
    sys.exit(0)