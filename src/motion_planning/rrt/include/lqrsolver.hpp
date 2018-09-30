#ifndef LQRSOLVER_HPP
#define LQRSOLVER_HPP

#include <iostream>
#include <vector>
#include "integrator.hpp"
#include "statespace.hpp"
#include "lyapunovsolver.hpp"
#include "schursorted.hpp"

template <typename Type, int n, int p, int q, typename StateSpace>
class LQRSolver
{
public:
  typedef Eigen::Matrix<Type,p,1> InputType;
  typedef Eigen::Matrix<Type,n,1> StateType;
  typedef Eigen::Matrix<Type,n,n> WeightQMat;
  typedef Eigen::Matrix<Type,p,p> WeightRMat;
  typedef Eigen::Matrix<Type,p,n> GainMat;
  typedef WeightQMat SysMat;

public:
  LQRSolver(const StateSpace &system_,
            WeightQMat Q = WeightQMat::Identity(),
            WeightRMat R = WeightRMat::Identity())
    : system(system_),
      Q(Q),
      R(R)
  { /*compute();*/ }

  // solve K using newton method for CARE
  template<int MaxIter=1000>
  void compute() {
    typedef Eigen::Matrix<Type,p,n> FeedbackMat;
    typedef Eigen::Matrix<Type,2*n,2*n> Mat2n;

    const auto& B = system.B;
    const auto& A = system.A;

    // solve riccati equation using schur decomposition
    Mat2n z;
    auto AT = A.transpose();
    auto G = -B*R.inverse()*B.transpose();
    z <<  A,   G,
          -Q,  -AT;
    auto schur_pair = ordschur(z);
    auto U = std::get<1>(schur_pair);
    auto S = std::get<0>(schur_pair);
    auto u11 = U.block(0,0,n,n);
    auto u21 = U.block(n,0,n,n);
    P = u21*u11.inverse();

    // compute optimal gain
    K = R.inverse()*B.transpose()*P;

#ifdef TEST
    std::cout << "[LQR by schur decomp]" << std::endl;
    std::cout << "matrix G :\n" << G << std::endl;
    std::cout << "matrix A :\n" << A << std::endl;
    std::cout << "matrix B :\n" << B << std::endl;
    std::cout << "matrix P :\n" << P << std::endl;
    std::cout << "matrix U :\n" << U << std::endl;
    std::cout << "matrix K :\n" << K << std::endl;
    std::cout << "[LQR by schur decomp]" << std::endl;
#endif
  }

  // solve control input for a given state
  InputType solve(const StateType &x) const
  {
    return -K*x;
  }
public:
  const StateSpace &system;
  Eigen::Matrix<Type,n,n> P;
  WeightQMat Q;
  WeightRMat R;
  GainMat K;
};

#endif // LQRSOLVER_HPP
