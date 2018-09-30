#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sympy

def generate_controller(mA, mB) :
  r, t, b, tau = sympy.symbols('r t b tau')
  #r, t, po, vo, pf, vf, b, tau = sympy.symbols('r t po vo pf vf b tau')

  n = len(mA)
  A = sympy.Matrix(mA)
  B = sympy.Matrix(mB)
  rlist = []
  for i in range(len(mB[0])) :
    rv = []
    for j in range(len(mB[0])) :
      rv.append(0)
    rv[i] = r
    rlist.append(rv)
  # R = sympy.Identity()
  R = sympy.Matrix(rlist)
  cmp_mat = (A.row_join(B*R.inv()*B.transpose())).col_join(sympy.Matrix.zeros(A.rows,A.cols).row_join(-A.transpose()))
  exp_cmp_mat = sympy.exp(cmp_mat*t)
  print R, '\n', cmp_mat
  [cmp_P, cmp_J] = cmp_mat.jordan_form()

  x = []
  for i in range(n) :
    x.append(sympy.symbols('x%s'%i))

  xi_list = []
  xf_list = []
  if_var_str = 'Type '
  if_set_str = ''
  for i in range(n) :
    if_var_str = if_var_str + 'x%si'%i + ', ' + 'x%sf'%i
    if_set_str = if_set_str + 'x%si = xi(%s); x%sf = xf(%s); ' %(i,i,i,i)
    if i < n-1 : 
      if_var_str = if_var_str + ', '
    xi_list.append([sympy.symbols('x%si'%i)])
    xf_list.append([sympy.symbols('x%sf'%i)])
  if_var_str = if_var_str + '; '

  xi = sympy.Matrix(xi_list)
  xf = sympy.Matrix(xf_list)
  #xo = sympy.Matrix([[po], [vo]])
  #xf = sympy.Matrix([[pf], [vf]])
  xbar = sympy.exp(A*t)*xi

  G = sympy.integrate(sympy.exp(A*(t-tau))*B*R.inv()*B.transpose()*sympy.exp(A.transpose()*(t-tau)), (tau, 0, t))
  #G = sympy.Matrix([[t**3/(3*r), t**2/(2*r)],[t**2/(2*r), t/r]])
  d = G.inv() * (xf - xbar)

  t1 = (2*A*xf).transpose()*d
  t2 = d.transpose()*B*R.inv()*B.transpose()*d
  c = sympy.Matrix([t]) + (xf -xbar).transpose() * G.inv() * (xf - xbar)
  dc = sympy.Matrix([1])- t1 - t2

  aabb = []
  d_aabb = []
  for i in range(n) :
    aabb.append([xbar[i]-sympy.sqrt(G[i,i]*(b-t)), xbar[i]+sympy.sqrt(G[i,i]*(b-t))])
    d_aabb.append([sympy.diff(aabb[i][0],t), sympy.diff(aabb[i][1], t)])

  #p_min = xbar[0]-sympy.sqrt(G[0,0]*(b-t))
  #p_max = xbar[0]+sympy.sqrt(G[0,0]*(b-t))
  #v_min = xbar[1]-sympy.sqrt(G[1,1]*(b-t))
  #v_max = xbar[1]+sympy.sqrt(G[1,1]*(b-t))

  #dp_min = sympy.diff(p_min, t)
  #dp_max = sympy.diff(p_max, t)
  #dv_min = sympy.diff(v_min, t)
  #dv_max = sympy.diff(v_max, t)

  vr = sympy.Matrix([sympy.pi**2*(G*(b-t)).det()])
  eat = sympy.exp(A*t)
  [P, J] = A.jordan_form()

  C = sympy.Identity(n)

  exp_cmp_mat_str = 'cmp_eAt << '
  cmp_P_str = 'P << '
  cmp_J_str = 'D << '
  for i in range(cmp_J.rows) :
    for j in range(cmp_J.cols) :
      cmp_J_str += sympy.ccode(cmp_J[i,j])
      cmp_P_str += sympy.ccode(cmp_P[i,j])
      if (i+1)*(j+1)-1 < cmp_J.cols*cmp_J.rows-1 :
        cmp_J_str += ', '
        cmp_P_str += ', '
  cmp_P_str += ';'
  cmp_J_str += ';'

  gram_str = 'G << '
  eat_str = 'eAt << '
  p_str = 'P << '
  j_str = 'J << '
  a_str = 'A << '
  b_str = 'B << '
  c_str = 'C << '
  for i in range(n) :
    for j in range(n) :
      a_str = a_str + sympy.ccode(A[i,j])
      c_str = c_str + sympy.ccode(C[i,j])
      gram_str = gram_str + sympy.ccode(G[i,j])
      p_str = p_str + sympy.ccode(P[i,j])
      j_str = j_str + sympy.ccode(J[i,j])
      eat_str = eat_str + sympy.ccode(eat[i,j])
      if ((i+1)*(j+1) < n*n-1):
        gram_str = gram_str + ', '
        p_str = p_str + ', '
        j_str = j_str + ', '
        a_str = a_str + ', '
        c_str = c_str + ', '
        eat_str = eat_str + ', '
  for i in range(B.rows) :
    for j in range(B.cols) :
      b_str = b_str + sympy.ccode(B[i,j])
      if (i+1)*(j+1)-1 < B.rows*B.cols-1 :
        b_str = b_str + ', '
  gram_str = gram_str + ';'
  a_str = a_str + ';'
  b_str = b_str + ';'
  c_str = c_str + ';'
  p_str = p_str + ';'
  j_str = j_str + ';'
  eat_str = eat_str + ';'
  jordan_str = p_str + '\n' + j_str
  cost_str = 'cost = %s;' % sympy.ccode(c[0])
  dc_str = 'd_cost = %s;' % sympy.ccode(dc[0])
  vr_str = 'vr = %s;' % sympy.ccode(vr[0])
  aabb_str = 'aabb << '
  d_aabb_str = 'd_aabb << '
  for i in range(n) :
    for j in range(2) :
      aabb_str = aabb_str + sympy.ccode(aabb[i][j])
      d_aabb_str = d_aabb_str + sympy.ccode(d_aabb[i][j])
      if ((i+1)*(j+1) < n*2-1) :
        aabb_str = aabb_str + ', '
        d_aabb_str = d_aabb_str + ', '
  aabb_str = aabb_str + ';'
  d_aabb_str = d_aabb_str + ';'
  # print gram_str
  # print cost_str
  # print dc_str
  # print vr_str
  # print aabb_str
  # print d_aabb_str
  return [gram_str,jordan_str,eat_str,cost_str,dc_str,aabb_str,d_aabb_str,vr_str, if_var_str, if_set_str, a_str, b_str, c_str, cmp_J_str, cmp_P_str]

def generate_cpp(model_name, dim, u_dim, g, jordan, eat, c, dc, aabb, d_aabb, vr, if_var, if_set, a_str, b_str, c_str, cmp_J_str, cmp_P_str) :
  code_str = str(
    "#ifndef %s_HPP\n" % model_name.upper() +
    "#define %s_HPP\n" % model_name.upper() +
    "#include \"statespace.hpp\"\n"
    "#include \"statespacesolver.hpp\"\n"
    "#include \"lqrsolver.hpp\"\n"
    "#include \"fixedtimelqr.hpp\"\n"
    "#include \"feedbackcontroller.hpp\"\n"
    "namespace Models {\n"
    "#define SYS_N %s\n" % dim +
    "#define SYS_P %s\n" % u_dim +
    "#define SYS_Q %s\n" % dim +
    "\n"
    "const int n = SYS_N;\n"
    "const int p = SYS_P;\n"
    "const int q = SYS_Q;\n"
    "\n"
    "typedef double Type;\n"
    '  Type r = 1.0;\n'
    "\n"
    'struct %sClosedExpm {\n' % model_name +
    'Eigen::Matrix<Type,SYS_N,SYS_N> operator()(Type t) const\n'
    '{\n'
    '  Eigen::Matrix<Type,SYS_N,SYS_N> eAt;\n'
    '  %s\n' % eat +
    '  return eAt;\n'
    '}\n'
    '};\n'
    "typedef StateSpace<Type,SYS_N,SYS_P,SYS_Q> %sSS;\n" % model_name +
    "typedef StateSpaceSolver<Type,SYS_N,SYS_P,SYS_Q,%sSS> %sSolver;\n" % (model_name,model_name) +
    "typedef LQRSolver<Type,SYS_N,SYS_P,SYS_Q,%sSS> %sLQR;\n" % (model_name,model_name) +
    "typedef FeedbackController<Type,%sSolver,%sLQR> %sLQRControl;\n" % (model_name,model_name,model_name) +
    'struct %sJordanForm\n' % model_name + 
    '{\n'
    '  typedef std::tuple<%sSS::SystemMatrix,%sSS::SystemMatrix> Mat;\n' %(model_name,model_name) + 
    '  %sSS::SystemMatrix J, P;\n' % model_name + 
    '  %sJordanForm()\n' % model_name + 
    '  {\n'
    '    %s\n' % jordan +
    '  }\n'
    '  Mat operator()(){\n'
    '    return std::make_tuple(J,P);\n'
    '  }\n'
    '};\n'
    '%sSS %s;\n' %(model_name, model_name.lower()) +
    '%sSolver %s_solver(%s);\n' %(model_name, model_name.lower(), model_name.lower()) +
    '\n'
    '%sLQR %s_lqr(%s);\n' %(model_name, model_name.lower(), model_name.lower()) +
    '%sLQRControl %s_lqr_control(%s_solver, %s_lqr);\n' %(model_name, model_name.lower(), model_name.lower(), model_name.lower()) +
    'struct %sCost\n' % model_name +
    '{\n'
    '  Type operator()(const %sSS::StateType &xi, const %sSS::StateType &xf, const Type &t) const\n' %(model_name,model_name) +
    '  {\n'
    '    Type cost;\n'
    '    %s\n' % if_var +
    '    %s\n' % if_set +
    '    %s\n' % c +
    '    return cost;\n'
    '  }\n'
    '} %s_cost;\n' % model_name.lower() +
    'struct %sOptTimeDiff\n' % model_name + 
    '{\n'
    '  void set(const %sSS::StateType &xi, const %sSS::StateType &xf)\n' %(model_name,model_name) +
    '  {\n'
    '    %s\n' % if_set +
    '  }\n'
    '  Type operator()(const Type &t) const\n'
    '  {\n'
    '    Type d_cost;\n'
    '    %s\n' % dc +
    '    return d_cost;\n'
    '  }\n'
    '  %s\n' %if_var + 
    '} %s_opt_time_diff;\n' % model_name.lower() +
    'struct %sGramian {\n' %model_name +
    '  %sSS::SystemMatrix operator()(Type t) const\n' %model_name +
    '  {\n'
    '    %sSS::SystemMatrix G;\n' %model_name +
    '    %s\n' % g +
    '    return G;\n'
    '  }\n'
    '} %s_gram;\n' %model_name.lower() +
    'typedef StateSpace<Type,2*SYS_N,SYS_P,SYS_Q> %sSSComposite;\n' % model_name +
    'typedef FixedTimeLQR<%sSS,%sGramian> %sFixTimeLQR;\n' % (model_name,model_name,model_name) +
    'typedef OptimalTimeFinder<%sOptTimeDiff> %sOptTimeSolver;\n' % (model_name,model_name) +
    'typedef OptTrjSolver<%sCost,%sOptTimeSolver,%sFixTimeLQR,%sSS,%sGramian,%sSSComposite> %sTrajectorySolver;\n' \
    %(model_name,model_name,model_name,model_name,model_name,model_name,model_name) +
    '%sSSComposite %s_ss_cmp;\n' % (model_name, model_name.lower()) +
    '%sFixTimeLQR %s_ft_lqr(%s, %s, %s_gram);\n' \
    % (model_name, model_name.lower(), model_name.lower(), model_name.lower(), model_name.lower()) +
    '%sOptTimeSolver %s_opt_time_solver(%s_opt_time_diff);\n' \
    % (model_name, model_name.lower(), model_name.lower()) +
    '%sTrajectorySolver %s_trj_solver(%s_cost, %s_opt_time_solver,%s_ft_lqr,%s,%s_gram,%s_ss_cmp);\n' \
    % (model_name, model_name.lower(), model_name.lower(), model_name.lower(), model_name.lower(), model_name.lower(), model_name.lower(), model_name.lower()) +
    '\n'
    'void init_%s()\n' % model_name.lower() +
    '{\n'
    '  auto &ss = %s;\n' % model_name.lower() +
    '  %sJordanForm %s_jordan_form;\n' %(model_name, model_name.lower()) +
    '  ss.%s\n' % a_str + 
    '  ss.%s\n' % b_str + 
    '  ss.%s\n' % c_str + 
    '  auto t = %s_jordan_form();\n' % model_name.lower() +
    '  ss.D = std::get<0>(t);\n'
    '  ss.P = std::get<1>(t);\n'
    '  ss.P_inv = ss.P.inverse();\n'
    '\n'
    # '  Type r = 1.0;\n'
    '  auto R = %s_ft_lqr.R;\n' % model_name.lower() + 
    '  auto &ss_cmp = %s_ss_cmp;\n' % model_name.lower() +
    '  ss_cmp.A << ss.A,             ss.B*R.inverse()*ss.B.transpose(),\n'
    '              %sSS::SystemMatrix::Zero(), -ss.A.transpose();\n' % model_name + 
    '  ss_cmp.%s\n' % cmp_P_str + 
    '  ss_cmp.%s\n' % cmp_J_str +
    '  ss_cmp.P_inv = ss_cmp.P.inverse();\n'
    '\n'
    '  std::cout << "test expm :" << std::endl\n'
    '            << "A.expm(0.0) :"<< std::endl << ss.expm(0.0) << std::endl\n'
    '            << "A.expm(1.0) :"<< std::endl << ss.expm(1.0) << std::endl\n'
    '            << "A.expm(-1.0) :"<< std::endl << ss.expm(-1.0) << std::endl\n'
    '            << "A.expm(2.5) :"<< std::endl << ss.expm(2.5) << std::endl\n'
    '            << "A.expm(-2.5) :"<< std::endl << ss.expm(-2.5) << std::endl;\n'
    '\n'
    '  std::cout << "test composite matrix" << std::endl\n'
    '            << "ss_cmp.A :"<< std::endl << ss_cmp.A << std::endl\n'
    '            << "ss_cmp.expm(0.0) :"<< std::endl << ss_cmp.expm(0.0) << std::endl\n'
    '            << "ss_cmp.expm(-1.0) :"<< std::endl << ss_cmp.expm(-1.0) << std::endl\n'
    '            << "ss_cmp.expm(1.0) :"<< std::endl << ss_cmp.expm(1.0) << std::endl;\n'
    '}\n'
    '\n'
    '%sSS& get_%s()\n' % (model_name, model_name.lower()) +
    '{\n'
    'return %s;\n' %model_name.lower() +
    '}\n'
    '\n'
    '}\n'
    '\n'
    '#endif // MODELS_HPP\n'
  )
  return code_str

def generate_cpp_main(model_name) :
  src = (
    '#include <iostream>\n'
    '#include <fstream>\n'
    '\n'
    '//#define TEST\n'
    '#include "kronecker.hpp"\n'
    '#include "lyapunovsolver.hpp"\n'
    '#include "statespacesolver.hpp"\n'
    '#include "pythonembedded.hpp"\n'
    '#include "jordan.hpp"\n'
    '#include "lqrsolver.hpp"\n'
    '#include "%s.hpp"\n' % model_name.lower() +
    '#include "polynomial.hpp"\n'
    '\n'
    '//#define FEEDBACKCONTROLLER_DEBUG_TIME\n'
    '//#define FEEDBACKCONTROLLER_DEBUG_PRINT\n'
    '//#define FEEDBACKCONTROLLER_DEBUG_LOG\n'
    '#include "feedbackcontroller.hpp"\n'
    '\n'
    '#include <QApplication>\n'
    '#include "mainwindow.h"\n'
    '\n'
    'int main(int argc, char** argv)\n'
    '{\n'
    '  QApplication a(argc, argv);\n'
    '  Models::init_%s();\n' % model_name.lower() +
    '  auto %s = Models::get_%s();\n' % (model_name.lower(), model_name.lower()) +
    '  auto &%sft_lqr = Models::%s_ft_lqr;\n' % (model_name.lower(), model_name.lower()) +
    '  // auto xf = Models::%sSS::StateType::Identity();\n' % (model_name) +
    '  // auto x0 = Models::%sSS::StateType::Zero();\n' % (model_name) +
    '  auto& opt_trajectory = Models::%s_trj_solver;\n' % model_name.lower() +
    '\n'
    '  MainWindow w(Models::n, Models::p, MainWindow::FREE_FINAL_TIME);\n'
    '  w.readSystem<double,Models::n>(%s.A);\n'% model_name.lower() +
    '\n'
    '  // callback for w.set_initial push button\n'
    '  w.init_cb = [&](\n'
    '      std::vector<double> values,\n'
    '      std::vector<std::vector<double>>,\n'
    '      std::vector<double>)\n'
    '  {\n'
    '    std::vector<double> xi_vec, xf_vec;\n'
    '    std::vector<Models::%sSS::StateType> trj;\n' % model_name +
    '    std::vector<Models::%sSS::InputType> inputs;\n' % model_name +
    '    std::vector<double> time;\n'
    '    xi_vec.insert(xi_vec.begin(),values.begin(),values.begin()+values.size()/2);\n'
    '    xf_vec.insert(xf_vec.begin(),values.begin()+values.size()/2,values.end());\n'
    '    Models::%sSS::StateType xi(xi_vec.data()), xf(xf_vec.data());\n' % model_name +
    '    auto trajectory = opt_trajectory.solve(xi,xf);\n'
    '    std::stringstream ss;\n'
    '    auto c = opt_trajectory.cost(xi,xf);\n'
    '    ss << "cost : " << std::get<1>(c) << ", time : " << std::get<0>(c);\n'
    '    w.log(ss.str());\n'
    '    for(const auto& t : trajectory) {\n'
    '      trj.push_back(std::get<1>(t));\n'
    '      time.push_back(std::get<0>(t));\n'
    '      inputs.push_back(std::get<2>(t));\n'
    '    }\n'
    '    w.plotStates<double,Models::n>(trj,time);\n'
    '    w.plotInputs<double,Models::p>(inputs,time);\n'
    '  };\n'
    '\n'
    '  // dont do anything\n'
    '  w.system_cb = [&](std::vector<double> sys_values) {\n'
    '    w.readSystem<double,Models::n>(%s.A);\n' % model_name.lower() +
    '  };\n'
    '\n'
    '  w.showMaximized();\n'
    '  return a.exec();\n'
    '}\n'
  )
  return src
