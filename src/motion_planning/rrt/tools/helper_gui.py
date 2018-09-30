#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtWidgets, QtCore, QtGui
from helper import *

class HelperGUI(object) :
  def __init__(self, **kwargs) :
    self.dim = 2
    self.u_dim = 1

    self.widgets = QtWidgets.QWidget()
    self.text = QtWidgets.QTextEdit()
    self.text_main = QtWidgets.QTextEdit()
    self.text_group = QtWidgets.QGroupBox("output")
    self.text_layout = QtWidgets.QGridLayout()

    self.tab = QtWidgets.QTabWidget()
    self.tab.insertTab(0,self.text, "models")
    self.tab.insertTab(1,self.text_main, "main")

    self.text_layout.addWidget(self.tab,0,0)
    self.text_group.setLayout(self.text_layout)
    self.cfg_layout = QtWidgets.QGridLayout()
    self.main_layout = QtWidgets.QGridLayout()

    self.main_layout.addWidget(self.text_group,0,0)
    self.dim_box = QtWidgets.QSpinBox()
    self.u_dim_box = QtWidgets.QSpinBox()
    self.model_name_layout = QtWidgets.QGridLayout()
    self.model_name = QtWidgets.QLineEdit()
    self.model_name.setMaximumWidth(200)

    dim_groupbox = QtWidgets.QGroupBox("dimension")
    dim_layout = QtWidgets.QGridLayout()
    dim_layout.addWidget(QtWidgets.QLabel('state dimension : '), 0, 0)
    dim_layout.addWidget(self.dim_box, 0, 1)
    dim_layout.addWidget(QtWidgets.QLabel('input dimension : '), 1, 0)
    dim_layout.addWidget(self.u_dim_box, 1, 1)
    dim_groupbox.setLayout(dim_layout)
    model_label = QtWidgets.QLabel('Model Name ')
    model_label.setMaximumWidth(100)

    self.model_name_layout.addWidget(model_label,0,0)
    self.model_name_layout.addWidget(self.model_name,0,1)
    self.cfg_layout.addLayout(self.model_name_layout, 0, 0)
    self.cfg_layout.addWidget(dim_groupbox, 1, 0)
    self.main_layout.addLayout(self.cfg_layout, 0, 1)
    self.widgets.setLayout(self.main_layout)

    self.sboxes = []
    self.dynamic_group = QtWidgets.QGroupBox("Dynamic Matrix (A)")
    self.dynamic_group_layout = QtWidgets.QGridLayout()
    self.u_sboxes = []
    self.input_group = QtWidgets.QGroupBox("Input Matrix (B)")
    self.input_group_layout = QtWidgets.QGridLayout()

    self.dim_box.setValue(self.dim)
    self.u_dim_box.setValue(self.u_dim)

    self.setSysLayout()

    self.save_model_btn = QtWidgets.QPushButton("save model")
    self.save_main_btn = QtWidgets.QPushButton("save main")
    self.gen_btn = QtWidgets.QPushButton("generate!")
    self.gen_btn.clicked.connect(self.generate)
    self.cfg_layout.addWidget(self.gen_btn,4,0)
    self.cfg_layout.addWidget(self.save_model_btn,5,0)
    self.cfg_layout.addWidget(self.save_main_btn,6,0)

    self.save_model_btn.clicked.connect(self.save_model)
    self.save_main_btn.clicked.connect(self.save_main)
    self.dim_box.valueChanged.connect(self.setSysLayout)
    self.u_dim_box.valueChanged.connect(self.setSysLayout)

  def save_model(self) :
    filename = QtWidgets.QFileDialog.getSaveFileName()
    src = self.text.toPlainText()
    print filename
    if filename[0] :
      with open(filename[0],'w+') as f:
        f.write(src)
  
  def save_main(self) :
    filename = QtWidgets.QFileDialog.getSaveFileName()
    src = self.text_main.toPlainText()
    print filename
    if filename[0] :
      with open(filename[0],'w+') as f:
        f.write(src)

  def generate(self) :
    A = []
    B = []
    for i in range(self.dim) :
      av = []
      bv = []
      for b in range(self.u_dim) :
        bv.append(self.u_sboxes[i*self.u_dim+b].value())
      for j in range(self.dim) :
        av.append(self.sboxes[i*self.dim+j].value())
      A.append(av)
      B.append(bv)

    dim = self.dim
    u_dim = self.u_dim
    model_name = self.model_name.text()

    def html_color(str, color) :
      return '<span style="color:%s;">%s</span>' %(color,str)
    def replace_color(str, key, color) :
      return str.replace(key, html_color(key,color))
    def parse_cpp (str) :
      kwords = {
        '#ifndef ' : 'violet',
        '#define ' : 'violet',
        '#include ' : 'violet',
        'namespace ' : 'blue',
        'const ' : 'blue',
        'int ' : 'blue',
        'typedef ' : 'blue',
        'double ' : 'blue',
        'struct ' : 'blue',
        'operator' : 'violet',
        'return ' : 'violet',
        'auto ' : 'blue',
        '#endif ' : 'violet',
        'StateSpaceSolver' : 'brown',
        'StateSpace' : 'brown',
        'LQRSolver' : 'brown',
        'FeedbackController' : 'brown',
        'FixedTimeLQR' : 'brown',
        'OptimalTimeFinder' : 'brown',
        'OptTrjSolver' : 'brown',
        '%sClosedExpm'%model_name : 'magenta',
        '%sSSComposite'%model_name : 'magenta',
        '%sSS'%model_name : 'magenta',
        '%sSolver'%model_name : 'magenta',
        '%sLQRControl'%model_name : 'magenta',
        '%sLQR'%model_name : 'magenta',
        '%sJordanForm'%model_name : 'magenta',
        '%sTimeOptDiff'%model_name : 'magenta',
        '%sGramian'%model_name : 'magenta',
        '%sFixTimeLQR'%model_name : 'magenta',
        '%sOptTimeSolver'%model_name : 'magenta',
        '%sTrajectorySolver'%model_name : 'magenta'
      }
      str = str.replace('<','&#60;').replace('>','&#62;').replace('\n','<br>')
      for k, v in kwords.items() :
        str = replace_color(str,k,v)
      return str

    [g, jordan, eat, c, dc, aabb, d_aabb, vr, if_var, if_set, a_str, b_str, c_str, cmp_J_str, cmp_P_str] = generate_controller(A,B)
    code_str = generate_cpp(model_name, dim, u_dim, g, jordan, eat, c, dc, aabb, d_aabb, vr, if_var, if_set, a_str, b_str, c_str, cmp_J_str, cmp_P_str)
    code_src = generate_cpp_main(model_name) 

    code_str = parse_cpp(code_str)
    code_src = parse_cpp(code_src)
    
    self.text.append(code_str)
    self.text_main.append(code_src)

  def show(self) :
    self.widgets.showMaximized()

  def setSysLayout(self) :
    self.dim = self.dim_box.value()
    self.u_dim = self.u_dim_box.value()
    if len(self.sboxes) :
      for s in self.sboxes :
        s.deleteLater()
        self.dynamic_group_layout.removeWidget(s)
    if len(self.u_sboxes) :
      for s in self.u_sboxes :
        s.deleteLater()
        self.input_group_layout.removeWidget(s)
    del self.sboxes[:]
    del self.u_sboxes[:]

    for i in range(self.dim) :
      for j in range(self.dim) :
        self.sboxes.append(QtWidgets.QSpinBox())
        self.dynamic_group_layout.addWidget(self.sboxes[-1],i,j)
    self.dynamic_group.setLayout(self.dynamic_group_layout)

    for i in range(self.dim) :
      for j in range(self.u_dim) :
        self.u_sboxes.append(QtWidgets.QSpinBox())
        self.input_group_layout.addWidget(self.u_sboxes[-1],i,j)
    self.input_group.setLayout(self.input_group_layout)

    if self.cfg_layout.rowCount() < 3 :
      self.cfg_layout.addWidget(self.dynamic_group, 2, 0)
    
    if self.cfg_layout.rowCount() < 4 :
      self.cfg_layout.addWidget(self.input_group, 3, 0)