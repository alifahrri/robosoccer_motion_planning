#ifndef SCHURSORTED_HPP
#define SCHURSORTED_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include "lapacke.h"

int eigenval_selector(const double *re, const double *im)
{
  if(*re < 0.0)
    return 1;
  else return 0;
}

// computes A = Z*T*Z^transpose
template<int N>
std::tuple<Eigen::Matrix<double,N,N>,Eigen::Matrix<double,N,N>>
ordschur(const Eigen::Matrix<double,N,N> &m)
{
  std::tuple<Eigen::Matrix<double,N,N>,Eigen::Matrix<double,N,N>> res;
  double a[N*N];
  double wr[N*N]; double wi[N*N]; double vs[N*N];
  for(int i=0; i<N*N; i++) a[i] = m.data()[i];
  int sdim[1];
  const int lda = N;
  const int ldvs = N;
  int (*select_ptr)(const double*, const double*) = eigenval_selector;
  LAPACKE_dgees(LAPACK_COL_MAJOR,'V','S',select_ptr,N,a,lda,sdim,wr,wi,vs,ldvs);
  std::get<0>(res) = Eigen::Matrix<double,N,N>(a); // T matrix
  std::get<1>(res) = Eigen::Matrix<double,N,N>(vs);
#ifdef TEST
#endif
  return res;

}

#endif // SCHURSORTED_HPP
