import velocitypublisher as velpub
import teleopdialog as dialog
import math
from PyQt5 import QtWidgets, QtGui, QtCore

N_ROBOT = 5

class MasterVelocity(QtWidgets.QGraphicsItem) :
    def __init__(self, *args, **kwargs):
        QtWidgets.QGraphicsItem.__init__(self, None)
        self.vx = 0.0
        self.vy = 0.0
        self.point = QtCore.QPointF(0.0,0.0)
        self.rotation_point = QtCore.QPointF(0.0,-125.0)
        self.w = 0.0

    def paint(self, painter, option, style) :
        painter.setPen(QtGui.QColor('blue'))
        painter.drawEllipse(-100.0, -100.0, 200.0, 200.0)
        painter.setPen(QtGui.QColor('green'))
        painter.drawEllipse(-125.0, -125.0, 250.0, 250.0)
        painter.setPen(QtGui.QColor('red'))
        painter.setBrush(QtGui.QColor('red'))
        arc_angle = QtCore.QLineF(QtCore.QPointF(0.0,0.0),self.rotation_point).angle() - 90.0
        rect = QtCore.QRectF(-125.0, -125.0, 250.0, 250.0)
        if arc_angle > 180.0 :
            span = 360 - arc_angle
            arc_angle = arc_angle - 360.0 + 90
            painter.drawArc(rect, arc_angle * 16, span)
        else :
            painter.drawArc(rect, 90.0 * 16, arc_angle * 16)
        painter.drawEllipse(self.point, 5.0, 5.0)
        painter.setPen(QtGui.QColor('green'))
        painter.setBrush(QtGui.QColor('green'))
        painter.drawEllipse(self.rotation_point, 5.0, 5.0)
    
    def mousePressEvent(self, event) :
        if event.button() == QtCore.Qt.LeftButton :
            self.point = event.pos()
            self.vx = self.point.x()
            self.vy = self.point.y()
        elif event.button() == QtCore.Qt.RightButton :
            pos = QtCore.QLineF(QtCore.QPointF(0.0,0.0), event.pos())
            pos.setLength(125.0)
            self.w = math.radians(pos.angle())
            self.rotation_point = pos.p2()
        if not (self.scene() is None) :
            self.scene().update()

    def mouseMoveEvent(self, event) :
        if event.button() == QtCore.Qt.LeftButton :
            self.point = event.pos()
            self.vx = self.point.x()
            self.vy = self.point.y()
        elif event.button() == QtCore.Qt.RightButton :
            pos = QtCore.QLineF(QtCore.QPointF(0.0,0.0), event.pos())
            pos.setLength(125.0)
            self.w = math.radians(pos.angle())
            self.rotation_point = pos
        if not (self.scene() is None) :
            self.scene().update()

    def boundingRect(self) :
        return QtCore.QRectF(-150,-150,300,300)

class TeleopGUI(object) :
    def __init__(self, *args, **kwargs):
        self.widget = QtWidgets.QWidget()
        self.ui = dialog.Ui_Dialog()
        self.ui.setupUi(self.widget)
        self.bot_velpub = []
        self.foe_velpub = []
        self.master_vel = MasterVelocity()
        self.scene = QtWidgets.QGraphicsScene(-200.0,-200.0,400,400,self.widget)
        self.scene.addItem(self.master_vel)
        self.ui.graphicsView.setScene(self.scene)
        self.ui.graphicsView.setRenderHint(QtGui.QPainter.Antialiasing)
        keys = ['stop', 'master', 'circular']
        for i in range(N_ROBOT) :
            bot_topic = '/nubot'+str(i+1)+'/nubotcontrol/velcmd'
            foe_topic = '/rival'+str(i+1)+'/nubotcontrol/velcmd'
            self.bot_velpub.append(velpub.VelocityPublisher(bot_topic))
            self.foe_velpub.append(velpub.VelocityPublisher(foe_topic))
        for key in keys :
            self.ui.bot_comboBox_1.addItem(key)
            self.ui.bot_comboBox_2.addItem(key)
            self.ui.bot_comboBox_3.addItem(key)
            self.ui.bot_comboBox_4.addItem(key)
            self.ui.bot_comboBox_5.addItem(key)
            self.ui.foe_comboBox_1.addItem(key)
            self.ui.foe_comboBox_2.addItem(key)
            self.ui.foe_comboBox_3.addItem(key)
            self.ui.foe_comboBox_4.addItem(key)
            self.ui.foe_comboBox_5.addItem(key)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)
    
    def update(self) :
        bot_config, foe_config = self.get_gui_config()
        mvx, mvy, w = self.get_master_vel()
        for i in range(N_ROBOT) :
            enabled = (bot_config[0][i], foe_config[0][i])
            key = (bot_config[1][i], foe_config[1][i])
            vm = (bot_config[2][i], foe_config[2][i])
            angle_rate = (bot_config[3][i], foe_config[3][i])
            self.bot_velpub[i].update(enabled[0], key=key[0], vmax=vm[0], angle_rate=angle_rate[0], vx=mvx, vy=mvy, w=w)
            self.foe_velpub[i].update(enabled[1], key=key[1], vmax=vm[1], angle_rate=angle_rate[1], vx=mvx, vy=mvy, w=w)
            self.bot_velpub[i].publish()
            self.foe_velpub[i].publish()

    def get_gui_config(self) :
        bot_vm = []
        foe_vm = []
        bot_key = []
        foe_key = []
        bot_rate = []
        foe_rate = []
        bot_checkboxes = []
        foe_checkboxes = []
        bot_checkboxes.append(self.ui.bot_checkBox_1.isChecked())
        bot_checkboxes.append(self.ui.bot_checkBox_2.isChecked())
        bot_checkboxes.append(self.ui.bot_checkBox_3.isChecked())
        bot_checkboxes.append(self.ui.bot_checkBox_4.isChecked())
        bot_checkboxes.append(self.ui.bot_checkBox_5.isChecked())
        foe_checkboxes.append(self.ui.foe_checkBox_1.isChecked())
        foe_checkboxes.append(self.ui.foe_checkBox_2.isChecked())
        foe_checkboxes.append(self.ui.foe_checkBox_3.isChecked())
        foe_checkboxes.append(self.ui.foe_checkBox_4.isChecked())
        foe_checkboxes.append(self.ui.foe_checkBox_5.isChecked())
        bot_vm.append(self.ui.bot_doubleSpinBox_1.value())
        bot_vm.append(self.ui.bot_doubleSpinBox_2.value())
        bot_vm.append(self.ui.bot_doubleSpinBox_3.value())
        bot_vm.append(self.ui.bot_doubleSpinBox_4.value())
        bot_vm.append(self.ui.bot_doubleSpinBox_5.value())
        foe_vm.append(self.ui.foe_doubleSpinBox_1.value())
        foe_vm.append(self.ui.foe_doubleSpinBox_2.value())
        foe_vm.append(self.ui.foe_doubleSpinBox_3.value())
        foe_vm.append(self.ui.foe_doubleSpinBox_4.value())
        foe_vm.append(self.ui.foe_doubleSpinBox_5.value())
        bot_key.append(self.ui.bot_comboBox_1.currentText())
        bot_key.append(self.ui.bot_comboBox_2.currentText())
        bot_key.append(self.ui.bot_comboBox_3.currentText())
        bot_key.append(self.ui.bot_comboBox_4.currentText())
        bot_key.append(self.ui.bot_comboBox_5.currentText())
        foe_key.append(self.ui.bot_comboBox_1.currentText())
        foe_key.append(self.ui.bot_comboBox_2.currentText())
        foe_key.append(self.ui.bot_comboBox_3.currentText())
        foe_key.append(self.ui.bot_comboBox_4.currentText())
        foe_key.append(self.ui.bot_comboBox_5.currentText())
        bot_rate.append(self.ui.bot_angle_rate_doubleSpinBox_1.value())
        bot_rate.append(self.ui.bot_angle_rate_doubleSpinBox_2.value())
        bot_rate.append(self.ui.bot_angle_rate_doubleSpinBox_3.value())
        bot_rate.append(self.ui.bot_angle_rate_doubleSpinBox_4.value())
        bot_rate.append(self.ui.bot_angle_rate_doubleSpinBox_5.value())
        foe_rate.append(self.ui.foe_angle_rate_doubleSpinBox_1.value())
        foe_rate.append(self.ui.foe_angle_rate_doubleSpinBox_2.value())
        foe_rate.append(self.ui.foe_angle_rate_doubleSpinBox_3.value())
        foe_rate.append(self.ui.foe_angle_rate_doubleSpinBox_4.value())
        foe_rate.append(self.ui.foe_angle_rate_doubleSpinBox_5.value())
        return (bot_checkboxes, bot_key, bot_vm, bot_rate), (foe_checkboxes, foe_key, foe_vm, foe_rate)

    def get_master_vel(self) :
        return self.master_vel.vx, self.master_vel.vy, self.master_vel.w