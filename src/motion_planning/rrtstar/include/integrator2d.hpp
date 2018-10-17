#ifndef INTEGRATOR2D_HPP
#define INTEGRATOR2D_HPP

#include "statespace.hpp"
#include "statespacesolver.hpp"
#include "fixedtimelqr.hpp"
//#include "lqrsolver.hpp"
//#include "feedbackcontroller.hpp"

#ifdef GPU
#include <cuda.h>
#include <cuda_runtime.h>
#define ATTRIBUTE __host__ __device__
#else
#define ATTRIBUTE
#endif

namespace Models {
#define SYS_N 4
#define SYS_P 2
#define SYS_Q 4

constexpr int n = SYS_N;
constexpr int p = SYS_P;
constexpr int q = SYS_Q;

typedef double scalar;
constexpr scalar r = 1.0;

struct Integrator2DClosedExpm {
  ATTRIBUTE
  Eigen::Matrix<scalar,SYS_N,SYS_N> operator()(scalar t) const
  {
    Eigen::Matrix<scalar,SYS_N,SYS_N> eAt;
    eAt << 1, 0, t, 0, 0, 1, 0, t, 0, 0, 1, 0, 0, 0, 0, 1;
    return eAt;
  }
};
//typedef StateSpace<Type,SYS_N,SYS_P,SYS_Q> Integrator2DSS;
typedef StateSpace<scalar,SYS_N,SYS_P,SYS_Q,Integrator2DClosedExpm> Integrator2DSS;
typedef StateSpaceSolver<scalar,SYS_N,SYS_P,SYS_Q,Integrator2DSS> Integrator2DSolver;
//typedef LQRSolver<Type,SYS_N,SYS_P,SYS_Q,Integrator2DSS> Integrator2DLQR;
//typedef FeedbackController<Type,Integrator2DSolver,Integrator2DLQR> Integrator2DLQRControl;
struct Integrator2DJordanForm
{
#ifdef __CUDA_ARCH__
  struct Mat {
    ATTRIBUTE Mat() {}
    Integrator2DSS::SystemMatrix J, P;
  };
#else
  typedef std::tuple<Integrator2DSS::SystemMatrix,Integrator2DSS::SystemMatrix> Mat;
#endif
  Integrator2DSS::SystemMatrix J, P;
  ATTRIBUTE
  Integrator2DJordanForm()
  {
    P << 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1;
    J << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
  }
  ATTRIBUTE
  Mat operator()(){
#ifdef __CUDA_ARCH__
    Mat m;
    m.J = J;
    m.P = P;
    return m;
#else
    return std::make_tuple(J,P);
#endif
  }
};
Integrator2DSS integrator2d;
Integrator2DSolver integrator2d_solver(integrator2d);

//Integrator2DLQR integrator2d_lqr(integrator2d);
//Integrator2DLQRControl integrator2d_lqr_control(integrator2d_solver, integrator2d_lqr);

struct Integrator2DCost
{
  ATTRIBUTE
  scalar operator()(const Integrator2DSS::StateType &xi, const Integrator2DSS::StateType &xf, const scalar &t) const
  {
    scalar cost;
    scalar x0i, x0f, x1i, x1f, x2i, x2f, x3i, x3f;
    x0i = xi(0); x0f = xf(0); x1i = xi(1); x1f = xf(1); x2i = xi(2); x2f = xf(2); x3i = xi(3); x3f = xf(3);
    cost = t + (x2f - x2i)*(4*r*(x2f - x2i)/t - 6*r*(-t*x2i + x0f - x0i)/pow(t, 2)) + (x3f - x3i)*(4*r*(x3f - x3i)/t - 6*r*(-t*x3i + x1f - x1i)/pow(t, 2)) + (-6*r*(x2f - x2i)/pow(t, 2) + 12*r*(-t*x2i + x0f - x0i)/pow(t, 3))*(-t*x2i + x0f - x0i) + (-6*r*(x3f - x3i)/pow(t, 2) + 12*r*(-t*x3i + x1f - x1i)/pow(t, 3))*(-t*x3i + x1f - x1i);
    return cost;
  }
} integrator2d_cost;
struct Integrator2DOptTimeDiff
{
  ATTRIBUTE
  void set(const Integrator2DSS::StateType &xi, const Integrator2DSS::StateType &xf)
  {
    x0i = xi(0); x0f = xf(0); x1i = xi(1); x1f = xf(1); x2i = xi(2); x2f = xf(2); x3i = xi(3); x3f = xf(3);
  }
  ATTRIBUTE
  scalar operator()(const scalar &t) const
  {
    scalar d_cost;
    d_cost = -2*x2f*(-6*r*(x2f - x2i)/pow(t, 2) + 12*r*(-t*x2i + x0f - x0i)/pow(t, 3)) - 2*x3f*(-6*r*(x3f - x3i)/pow(t, 2) + 12*r*(-t*x3i + x1f - x1i)/pow(t, 3)) + 1 - pow(4*r*(x2f - x2i)/t - 6*r*(-t*x2i + x0f - x0i)/pow(t, 2), 2)/r - pow(4*r*(x3f - x3i)/t - 6*r*(-t*x3i + x1f - x1i)/pow(t, 2), 2)/r;
    return d_cost;
  }
  scalar x0i, x0f, x1i, x1f, x2i, x2f, x3i, x3f;
} integrator2d_opt_time_diff;
struct Integrator2DGramian {
  ATTRIBUTE
  Integrator2DSS::SystemMatrix operator()(scalar t) const
  {
    Integrator2DSS::SystemMatrix G;
    G << (1.0/3.0)*pow(t, 3)/r, 0, (1.0/2.0)*pow(t, 2)/r, 0, 0, (1.0/3.0)*pow(t, 3)/r, 0, (1.0/2.0)*pow(t, 2)/r, (1.0/2.0)*pow(t, 2)/r, 0, t/r, 0, 0, (1.0/2.0)*pow(t, 2)/r, 0, t/r;
    return G;
  }
} integrator2d_gram;
struct Integrator2DCmpClosedExpm {
  ATTRIBUTE Eigen::Matrix<scalar,2*SYS_N,2*SYS_N> operator()(scalar t) const
  {
    Eigen::Matrix<scalar,2*SYS_N,2*SYS_N> eAt;
    eAt << 1, 0, t, 0, -1.0/6.0*pow(t, 3)/r, 0, (1.0/2.0)*pow(t, 2)/r, 0, 0, 1, 0, t, 0, -1.0/6.0*pow(t, 3)/r, 0, (1.0/2.0)*pow(t, 2)/r, 0, 0, 1, 0, -1.0/2.0*pow(t, 2)/r, 0, t/r, 0, 0, 0, 0, 1, 0, -1.0/2.0*pow(t, 2)/r, 0, t/r, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, -t, 0, 1, 0, 0, 0, 0, 0, 0, -t, 0, 1;
    return eAt;
  }
};
//typedef StateSpace<Type,2*SYS_N,SYS_P,SYS_Q> Integrator2DSSComposite;
typedef StateSpace<scalar,2*SYS_N,SYS_P,SYS_Q,Integrator2DCmpClosedExpm> Integrator2DSSComposite;
typedef FixedTimeLQR<Integrator2DSS,Integrator2DGramian> Integrator2DFixTimeLQR;
typedef OptimalTimeFinder<Integrator2DOptTimeDiff> Integrator2DOptTimeSolver;
typedef OptTrjSolver<Integrator2DCost,Integrator2DOptTimeSolver,Integrator2DFixTimeLQR,Integrator2DSS,Integrator2DGramian,Integrator2DSSComposite> Integrator2DTrajectorySolver;
Integrator2DSSComposite integrator2d_ss_cmp;
Integrator2DFixTimeLQR integrator2d_ft_lqr(integrator2d, integrator2d, integrator2d_gram);
Integrator2DOptTimeSolver integrator2d_opt_time_solver(integrator2d_opt_time_diff);
Integrator2DTrajectorySolver integrator2d_trj_solver(integrator2d_cost, integrator2d_opt_time_solver,integrator2d_ft_lqr,integrator2d,integrator2d_gram,integrator2d_ss_cmp);

struct Integrator2D {
  typedef Integrator2DSS::StateType State;
  typedef Integrator2DSS::InputType Input;

  Integrator2D()
  {
    auto &ss = this->state_space;
    Integrator2DJordanForm integrator2d_jordan_form;
    ss.A << 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    ss.B << 0, 0, 0, 0, 1, 0, 0, 1;
    ss.C << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    auto t = integrator2d_jordan_form();
#ifndef __CUDA_ARCH__
    ss.D = std::get<0>(t);
    ss.P = std::get<1>(t);
#else
    ss.D = t.J;
    ss.P = t.P;
#endif
    ss.P_inv = ss.P.inverse();

    auto R = ft_lqr.R;
    auto &ss_cmp = this->composite_ss;
    ss_cmp.A << ss.A, ss.B*R.inverse()*ss.B.transpose(), Integrator2DSS::SystemMatrix::Zero(), -ss.A.transpose();
    ss_cmp.P << -1/r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0;
    ss_cmp.D << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    ss_cmp.P_inv = ss_cmp.P.inverse();
  }
  Integrator2DSS state_space;
  Integrator2DCost cost;
  Integrator2DGramian gramian;
  Integrator2DSSComposite composite_ss;
  Integrator2DOptTimeDiff opt_time_diff;
  // Integrator2DSolver ss_solver = Integrator2DSolver(state_space);
  // Integrator2DLQR integrator2d_lqr = Integrator2DLQR(state_space);
  // Integrator2DLQRControl integrator2d_lqr_control = Integrator2DLQRControl(ss_solver, integrator2d_lqr);
  Integrator2DFixTimeLQR ft_lqr = Integrator2DFixTimeLQR(state_space, state_space, gramian);
  Integrator2DOptTimeSolver opt_time_solver = Integrator2DOptTimeSolver(opt_time_diff);
  Integrator2DTrajectorySolver solver = Integrator2DTrajectorySolver(cost, opt_time_solver,ft_lqr,state_space,gramian,composite_ss);
};

#ifdef GPU
__host__
#endif
void init_integrator2d()
{
  auto &ss = integrator2d;
  Integrator2DJordanForm integrator2d_jordan_form;
  ss.A << 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
  ss.B << 0, 0, 0, 0, 1, 0, 0, 1;
  ss.C << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  auto t = integrator2d_jordan_form();
#ifndef __CUDA_ARCH__
    ss.D = std::get<0>(t);
    ss.P = std::get<1>(t);
#else
    ss.D = t.J;
    ss.P = t.P;
#endif
  ss.P_inv = ss.P.inverse();

  auto R = integrator2d_ft_lqr.R;
  auto &ss_cmp = integrator2d_ss_cmp;
  ss_cmp.A << ss.A, ss.B*R.inverse()*ss.B.transpose(),
      Integrator2DSS::SystemMatrix::Zero(), -ss.A.transpose();
  ss_cmp.P << -1/r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0;
  ss_cmp.D << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
  ss_cmp.P_inv = ss_cmp.P.inverse();

  /* std::cout << "test expm :" << std::endl
<< "A.expm(0.0) :"<< std::endl << ss.expm(0.0) << std::endl
<< "A.expm(1.0) :"<< std::endl << ss.expm(1.0) << std::endl
<< "A.expm(-1.0) :"<< std::endl << ss.expm(-1.0) << std::endl
<< "A.expm(2.5) :"<< std::endl << ss.expm(2.5) << std::endl
<< "A.expm(-2.5) :"<< std::endl << ss.expm(-2.5) << std::endl;

std::cout << "test composite matrix" << std::endl
<< "ss_cmp.A :"<< std::endl << ss_cmp.A << std::endl
<< "ss_cmp.expm(0.0) :"<< std::endl << ss_cmp.expm(0.0) << std::endl
<< "ss_cmp.expm(-1.0) :"<< std::endl << ss_cmp.expm(-1.0) << std::endl
<< "ss_cmp.expm(1.0) :"<< std::endl << ss_cmp.expm(1.0) << std::endl;
*/
}

Integrator2DSS& get_integrator2d()
{
  return integrator2d;
}

}

#endif // MODELS_HPP
