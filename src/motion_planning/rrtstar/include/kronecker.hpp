#ifndef KRONECKER_HPP
#define KRONECKER_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Eigenvalues>

template <typename Type, int m, int n, int p, int q>
Eigen::Matrix<Type, m*p, n*q>
kron(Eigen::Matrix<Type,m,n> A, Eigen::Matrix<Type,p,q> B)
{
  Eigen::Matrix<Type, m*p, n*q> res;
  Eigen::Matrix<Type,p,q> t[m*n];
  for(size_t i=0; i<m; i++)
    for(size_t j=0; j<n; j++) {
      auto tmp = A(i,j)*B;
      res.template block<p,q>(i*m,j*n) = tmp;
    }
  return res;
}

#endif // KRONECKER_HPP
