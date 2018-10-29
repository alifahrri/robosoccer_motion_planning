import velocitypublisher as velpub
import robotsubscriber as robosub
import teleopdialog as dialog
import modelspawner as spawner
import math
import rospy
import sys
import yaml
from PyQt5 import QtWidgets, QtGui, QtCore

N_ROBOT = 5

class LinesItem(QtWidgets.QGraphicsItem) :
    def __init__(self, *args, **kwargs):
        QtWidgets.QGraphicsItem.__init__(self,None)
        self.WIDTH = 2000
        self.HEIGHT = 1400
        self.STEP = 60
        self.h_lines = []
        self.v_lines = []
        self.waypoints = []
        WIDTH = self.WIDTH
        HEIGHT = self.HEIGHT
        STEP = self.STEP
        for i in range(WIDTH/STEP+1) :
            line = QtCore.QLineF(i*STEP-WIDTH/2,-HEIGHT/2,i*STEP-WIDTH/2,HEIGHT/2)
            self.v_lines.append(line)
        for i in range(HEIGHT/STEP+1) :
            line = QtCore.QLineF(-WIDTH/2,i*STEP-HEIGHT/2,WIDTH/2,i*STEP-HEIGHT/2)
            self.h_lines.append(line)
        
    def paint(self, painter, option, style) :
        painter.setPen(QtCore.Qt.green)
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawLines(self.v_lines)
        painter.drawLines(self.h_lines)
        for i in range(len(self.waypoints)) :
            p1 = self.waypoints[i]
            painter.setPen(QtCore.Qt.gray)
            painter.setBrush(QtCore.Qt.gray)
            painter.drawEllipse(p1, 10.0, 10.0)
            if i > 0 :
                p0 = self.waypoints[i-1]
                painter.drawLine(p0, p1)
            font = painter.font()
            font.setPixelSize(12)
            painter.setFont(font)
            painter.setPen(QtCore.Qt.green)
            painter.drawText(p1, str(i))

    def mousePressEvent(self, event) :
        self.waypoints.append(event.pos())
        if not (self.scene() is None) :
            self.scene().update()

    def mouseDoubleClickEvent(self, event) :
        del self.waypoints[:]
        if not (self.scene() is None) :
            self.scene().update()

    def boundingRect(self) :
        return QtCore.QRectF(-self.WIDTH/2,-self.HEIGHT/2,self.WIDTH,self.HEIGHT)

class MasterVelocity(QtWidgets.QGraphicsItem) :
    def __init__(self, *args, **kwargs):
        QtWidgets.QGraphicsItem.__init__(self, None)
        self.vx = 0.0
        self.vy = 0.0
        self.point = QtCore.QPointF(0.0,0.0)
        self.rotation_point = QtCore.QPointF(0.0, 125.0)
        self.w = 0.0

    def paint(self, painter, option, style) :
        painter.setPen(QtGui.QColor('blue'))
        painter.drawEllipse(-100.0, -100.0, 200.0, 200.0)
        painter.setPen(QtGui.QColor('green'))
        painter.drawEllipse(-125.0, -125.0, 250.0, 250.0)
        painter.setPen(QtGui.QColor('red'))
        painter.setBrush(QtGui.QColor('red'))
        painter.drawEllipse(self.point, 5.0, 5.0)
        painter.setPen(QtGui.QColor('green'))
        painter.setBrush(QtGui.QColor('green'))
        painter.drawEllipse(self.rotation_point, 5.0, 5.0)
        painter.rotate(-90.0)
        arc_angle = QtCore.QLineF(QtCore.QPointF(0.0,0.0),self.rotation_point).angle()
        rect = QtCore.QRectF(-125.0, -125.0, 250.0, 250.0)
        if arc_angle > 180.0 :
            span = 360 - arc_angle
            arc_angle = arc_angle - 360.0
            painter.drawArc(rect, arc_angle * 16, span)
        else :
            painter.drawArc(rect, 0.0 * 16, arc_angle * 16)
    
    def mousePressEvent(self, event) :
        self.mouse_btn = event.button()
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
        if self.mouse_btn == QtCore.Qt.LeftButton :
            self.point = event.pos()
            self.vx = self.point.x()
            self.vy = self.point.y()
        elif self.mouse_btn == QtCore.Qt.RightButton :
            pos = QtCore.QLineF(QtCore.QPointF(0.0,0.0), event.pos())
            pos.setLength(125.0)
            self.w = math.radians(pos.angle())
            self.rotation_point = pos.p2()
        if not (self.scene() is None) :
            self.scene().update()

    def boundingRect(self) :
        return QtCore.QRectF(-130,-130,260,260)

class TeleopGUI(QtWidgets.QMainWindow) :
    def __init__(self, *args, **kwargs):
        # main windown settings, include menut
        QtWidgets.QMainWindow.__init__(self)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.setWindowTitle("teleop main window")
        self.file_menu = QtWidgets.QMenu('&File', self)
        self.file_menu.addAction('&Quit', self.fileQuit,
                                 QtCore.Qt.CTRL + QtCore.Qt.Key_Q)
        self.menuBar().addMenu(self.file_menu)
        self.view_menu = QtWidgets.QMenu('&View', self)
        self.view_menu.addAction('&Spawn', self.spawn)
        self.menuBar().addSeparator()
        self.menuBar().addMenu(self.view_menu)

        # central widget settings
        self.widget = QtWidgets.QWidget()
        self.ui = dialog.Ui_Dialog()
        self.ui.setupUi(self.widget)
        self.ui.splitter.setSizes([700,0])
        self.ui.splitter_2.setSizes([0,700])
        self.bot_velpub, self.foe_velpub = [], []
        self.bot_sub, self.foe_sub = [], []
        self.master_vel = MasterVelocity()
        self.lines_item = LinesItem()
        ## scene settings
        self.scene = QtWidgets.QGraphicsScene(-200.0,-200.0,400,400,self.widget)
        self.scene.addItem(self.master_vel)
        self.ui.graphicsView.setScene(self.scene)
        self.ui.graphicsView.setRenderHint(QtGui.QPainter.Antialiasing)
        self.wp_scene = QtWidgets.QGraphicsScene(-900,-600,1800,1200)
        self.wp_scene.addItem(self.lines_item)
        self.ui.wp_graphicsView.setScene(self.wp_scene)
        self.ui.wp_graphicsView.setRenderHint(QtGui.QPainter.Antialiasing)
        FIELD_SCALE = 0.5
        self.ui.wp_graphicsView.scale(FIELD_SCALE, FIELD_SCALE)

        # prepare container
        self.combo_box = {'bot':[], 'foe':[]}
        self.check_box = {'bot':[], 'foe':[]}
        self.spin_box = {'bot':[], 'foe':[]}
        self.angle_rate_box = {'bot':[], 'foe':[]}
        # fill the box
        self.set_boxes()

        ### some control widget settings
        keys = ['stop', 'master', 'circular', 'waypoint']
        for i in range(N_ROBOT) :
            bot_topic = '/nubot'+str(i+1)+'/nubotcontrol/velcmd'
            foe_topic = '/rival'+str(i+1)+'/nubotcontrol/velcmd'
            self.bot_velpub.append(velpub.VelocityPublisher(bot_topic))
            self.foe_velpub.append(velpub.VelocityPublisher(foe_topic))
            bot_topic = '/nubot'+str(i+1)+'/omnivision/OmniVisionInfo'
            foe_topic = '/rival'+str(i+1)+'/omnivision/OmniVisionInfo'
            self.bot_sub.append(robosub.RobotItem(bot_topic, i+1))
            self.foe_sub.append(robosub.RobotItem(foe_topic, i+1, flip=-1))
            self.wp_scene.addItem(self.bot_sub[i])
            self.wp_scene.addItem(self.foe_sub[i])
        for key in keys :
            for box in self.combo_box['bot'] :
                box.addItem(key)
            for box in self.combo_box['foe'] :
                box.addItem(key)
        for i in range(len(self.bot_sub)) :
            self.ui.wp_comboBox.addItem('bot %s'%(i+1))
        for i in range(len(self.foe_sub)) :
            self.ui.wp_comboBox.addItem('foe %s'%(i+1))

        ## set connection and timer (for rospy check)
        self.ui.save_btn.clicked.connect(self.save_yaml)
        self.ui.load_btn.clicked.connect(self.load_yaml)
        self.ui.wp_setBtn.clicked.connect(self.setWaypoints)
        self.ui.wp_clearBtn.clicked.connect(self.clearWaypoints)
        self.widget.setWindowTitle('Teleoperation')
        self.setCentralWidget(self.widget)
        self.centralWidget().layout().setContentsMargins(0,0,0,0)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)

    def fileQuit(self):
        self.close()

    def closeEvent(self, ce):
        self.fileQuit()

    def spawn(self) :
        dialog = spawner.SpawnerDialog(self)
        dialog.show()

    def control(self) :
        bot_config, foe_config = self.get_gui_config()
        mvx, mvy, w = self.get_master_vel()
        for i in range(N_ROBOT) :
            enabled = (bot_config[0][i], foe_config[0][i])
            key = (bot_config[1][i], foe_config[1][i])
            vm = (bot_config[2][i], foe_config[2][i])
            angle_rate = (bot_config[3][i], foe_config[3][i])
            self.bot_velpub[i].update(enabled[0], key=key[0], vmax=vm[0], angle_rate=angle_rate[0], vx=mvx, vy=mvy, w=w)
            self.foe_velpub[i].update(enabled[1], key=key[1], vmax=vm[1], angle_rate=angle_rate[1], vx=mvx, vy=mvy, w=w)
            bot_err = self.bot_velpub[i].pidControl(self.bot_sub[i].target(), self.bot_sub[i].pose())
            foe_err = self.foe_velpub[i].pidControl(self.foe_sub[i].target(), self.foe_sub[i].pose())
            MIN_ERROR = 10.0
            if not (bot_err is None) :
                if bot_err[0] < MIN_ERROR :
                    self.bot_sub[i].nextTarget()
            if not (foe_err is None) :
                if foe_err[0] < MIN_ERROR :
                    self.foe_sub[i].nextTarget()
            self.bot_velpub[i].publish()
            self.foe_velpub[i].publish()
    
    def update(self) :
        self.control()
        self.wp_scene.update()
        if rospy.is_shutdown() :
            self.widget.close()
            self.fileQuit()

    def save_yaml(self) :
        f = QtWidgets.QFileDialog.getSaveFileName()
        waypoints = {'bot' : [], 'foe' : []}
        for i in range(len(self.bot_sub)) :
            waypoints['bot'].append([w for w in self.bot_sub[i].waypoints])
        for i in range(len(self.foe_sub)) :
            waypoints['foe'].append([w for w in self.foe_sub[i].waypoints])
        stream = file(f[0], 'w+')
        yaml.dump(waypoints, stream=stream)
        print yaml.dump(waypoints)

    def load_yaml(self) :
        self.clearWaypoints()
        f = QtWidgets.QFileDialog.getOpenFileName()
        stream = file(f[0], 'r')
        waypoints = yaml.load(stream)
        for i in range(len(waypoints['bot'])) :
            if len(waypoints['bot'][i]) :
                self.bot_sub[i].setWaypoints(waypoints['bot'][i])
        for i in range(len(waypoints['foe'])) :
            if len(waypoints['foe'][i]) :
                self.foe_sub[i].setWaypoints(waypoints['foe'][i])
        print waypoints

    def setWaypoints(self) :
        i = self.ui.wp_comboBox.currentIndex()
        if i < 0 :
            return
        wp = self.lines_item.waypoints
        if i < 5 :
            self.bot_sub[i].setWaypoints(wp)
        else :
            i = i % len(self.foe_sub)
            self.foe_sub[i].setWaypoints(wp)
        del self.lines_item.waypoints[:]
    
    def clearWaypoints(self) :
        i = self.ui.wp_comboBox.currentIndex()
        if i < 0 :
            return
        del self.lines_item.waypoints[:]
        if i < 5 :
            self.bot_sub[i].setWaypoints(None)
        else :
            i = i % len(self.foe_sub)
            self.foe_sub[i].setWaypoints(None)

    def get_gui_config(self) :
        bot_checkboxes = [box.isChecked() for box in self.check_box['bot']]
        foe_checkboxes = [box.isChecked() for box in self.check_box['foe']]
        bot_vm = [box.value() for box in self.spin_box['bot']]
        foe_vm = [box.value() for box in self.spin_box['foe']]
        bot_key = [box.currentText() for box in self.combo_box['bot']]
        foe_key = [box.currentText() for box in self.combo_box['foe']]
        bot_rate = [box.value() for box in self.angle_rate_box['bot']]
        foe_rate = [box.value() for box in self.angle_rate_box['foe']]
        return (bot_checkboxes, bot_key, bot_vm, bot_rate), (foe_checkboxes, foe_key, foe_vm, foe_rate)

    def get_master_vel(self) :
        return self.master_vel.vx, self.master_vel.vy, self.master_vel.w

    def set_boxes(self) :
        self.combo_box = {
            'bot' : [
                self.ui.bot_comboBox_1,
                self.ui.bot_comboBox_2,
                self.ui.bot_comboBox_3,
                self.ui.bot_comboBox_4,
                self.ui.bot_comboBox_5
            ],
            'foe' : [
                self.ui.foe_comboBox_1,
                self.ui.foe_comboBox_2,
                self.ui.foe_comboBox_3,
                self.ui.foe_comboBox_4,
                self.ui.foe_comboBox_5
            ]
        }

        self.check_box = {
            'bot' : [
                self.ui.bot_checkBox_1,
                self.ui.bot_checkBox_2,
                self.ui.bot_checkBox_3,
                self.ui.bot_checkBox_4,
                self.ui.bot_checkBox_5
            ],
            'foe' : [
                self.ui.foe_checkBox_1,
                self.ui.foe_checkBox_2,
                self.ui.foe_checkBox_3,
                self.ui.foe_checkBox_4,
                self.ui.foe_checkBox_5
            ]
        }

        self.spin_box = {
            'bot' : [
                self.ui.bot_doubleSpinBox_1,
                self.ui.bot_doubleSpinBox_2,
                self.ui.bot_doubleSpinBox_3,
                self.ui.bot_doubleSpinBox_4,
                self.ui.bot_doubleSpinBox_5
            ],
            'foe' : [
                self.ui.foe_doubleSpinBox_1,
                self.ui.foe_doubleSpinBox_2,
                self.ui.foe_doubleSpinBox_3,
                self.ui.foe_doubleSpinBox_4,
                self.ui.foe_doubleSpinBox_5
            ]
        }

        self.angle_rate_box = {
            'bot' : [
                self.ui.bot_angle_rate_doubleSpinBox_1,
                self.ui.bot_angle_rate_doubleSpinBox_2,
                self.ui.bot_angle_rate_doubleSpinBox_3,
                self.ui.bot_angle_rate_doubleSpinBox_4,
                self.ui.bot_angle_rate_doubleSpinBox_5
            ],
            'foe' : [
                self.ui.foe_angle_rate_doubleSpinBox_1,
                self.ui.foe_angle_rate_doubleSpinBox_2,
                self.ui.foe_angle_rate_doubleSpinBox_3,
                self.ui.foe_angle_rate_doubleSpinBox_4,
                self.ui.foe_angle_rate_doubleSpinBox_5
            ]
        }
