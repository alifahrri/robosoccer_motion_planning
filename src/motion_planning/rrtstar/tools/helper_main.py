import helper_gui
from helper import *
from PyQt5 import QtCore, QtGui, QtWidgets
import sys

if __name__ == '__main__' :
    app = QtWidgets.QApplication(sys.argv)
    generate_controller([[0,1],[0,0]],[[0],[1]])
    gui = helper_gui.HelperGUI()
    gui.show()
    sys.exit(app.exec_())