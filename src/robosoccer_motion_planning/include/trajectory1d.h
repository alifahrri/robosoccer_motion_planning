#ifndef TRAJECTORY1D_H
#define TRAJECTORY1D_H

#include <vector>
#include <iostream>

namespace Trajectory1D {

struct State {
  double w;
  double dw;
};

enum Case {
  INITIAL_STATE,
  ACCELERATION1,  //case 1
  ACCELERATION2_1,  //case 2.1 subcase 1
  ACCELERATION2_2,  //case 2.1 subcase 1
  CRUISING,       //case 2.2
  DECELERATION1,  //case 2.3
  DECELERATION2,  //case 3
};

struct Control {
  Case control_case;
  double effort;
  double time;
  State term;
};

class Controller
{
public:

  struct Control : std::vector<Trajectory1D::Control>
  {
    double final_time;
    double time_offset;
    double offset;
    double distance;
    double a_max;
    double v_max;
  };

  typedef std::pair<std::vector<State>,std::vector<double>> Trajectory;

public:
  Controller();
  void setLimit(double vmax=1.0, double amax=1.0);
  Controller::Control optimalControl(State init_state, double final_state, double& final_time);

  static State getState(const Controller::Control &ctrl, double time);
  static Trajectory getTrajectory(const Controller::Control& ctrl, double t0, double tf, int n);
  static Trajectory getTrajectory(const Controller::Control& ctrl, double t0, double tf, double dt);
  static double setMaxEffort(Controller::Control &ctrl, double amax);

private:
  std::string str(State &s);
  std::string str(Trajectory1D::Control &ctrl);
  void applyControl(Controller::Control& ctrl_seq, State& init_state, double final);
  void case1(Controller::Control& ctrl_seq, State& init_state, double final);
  void case21(Controller::Control& ctrl_seq, State& init_state, double final);
  void case22(Controller::Control& ctrl_seq, State& init_state, double final);
  void case23(Controller::Control& ctrl_seq, State& init_state, double final);
  void case3(Controller::Control& ctrl_seq, State& init_state, double final);
private:
  double v_max;
  double a_max;
};

// an angle controller consider shortest angle distance is not an euclidean distance
class AngleController : public Controller
{
public:
  struct AngleOffset
  {
    AngleOffset() {}
    double f_offset, t_offset;
  };
public:
  AngleController();
  Controller::Control angleControl(State init_state, double final_state, double& final_time);
public:
  static Trajectory getAngleTrajectory(const Control &ctrl, const AngleOffset &offset, double t0, double tf, int n);
  static Trajectory getAngleTrajectory(const Control &ctrl, const AngleOffset &offset, double t0, double tf, double dt);
public:
  double shortest_path(double from, double to, double &f_offset, double &t_offset);
  auto getOffset() { return offset; }
private:
  AngleOffset offset;
};

}

std::ostream& operator << (std::ostream& out, const Trajectory1D::Control &ctrl);

#endif // TRAJECTORY1D_H
