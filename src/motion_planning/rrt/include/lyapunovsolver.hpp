#ifndef LYAPUNOVSOLVER_HPP
#define LYAPUNOVSOLVER_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Eigenvalues>
#include "kronecker.hpp"

#ifdef TEST
#include <iostream>
#endif

template <typename Type, int n>
class LyapunovSolver
{
public:
  typedef Eigen::Matrix<Type,n,n> Mat;
public:
  LyapunovSolver(const Mat& A, const Mat &C) {
    Mat I = Mat::Identity();
    auto tmp = kron(I,A) + kron(A,I);
    auto c_vec = vec(C);
#ifdef TEST
    std::cout << "tmp:\n" << tmp << std::endl;
    std::cout << "c_vec:\n" << c_vec << std::endl;
#endif
    Eigen::HouseholderQR<Eigen::Matrix<Type,n*n,n*n>> solver(tmp);
    Eigen::Matrix<Type,n*n,1> x = solver.solve(c_vec);
    X = Mat(x.data());
  }
public:
  Mat X;
private:
  //helper function
  Eigen::Matrix<Type,n*n,1> vec(const Mat &m) {
    Eigen::Matrix<Type,n*n,1> res(m.data());
    return res;
  }
};

template <typename Type, int n>
Eigen::Matrix<Type,n,n>
lyapunov(const Eigen::Matrix<Type,n,n> &A, const Eigen::Matrix<Type,n,n> &C)
{
  typedef Eigen::Matrix<Type,n,n> Mat;
  Mat I = Mat::Identity();
  auto tmp = kron(I,A) + kron(A,I);
  Eigen::Matrix<Type,n*n,1> c_vec(C.data());
  Eigen::HouseholderQR<Eigen::Matrix<Type,n*n,n*n>> solver(tmp);
  Eigen::Matrix<Type,n*n,1> x = solver.solve(c_vec);
  Mat X = Mat(x.data());
#ifdef TEST
  std::cout << "[lyapunov]" << std::endl;
  std::cout << "tmp:\n" << tmp << std::endl;
  std::cout << "c_vec:\n" << c_vec << std::endl;
  std::cout << "X :\n" << X << std::endl;
  std::cout << "[lyapunov]" << std::endl;
#endif
  return X;
}
#endif // LYAPUNOVSOLVER_HPP
