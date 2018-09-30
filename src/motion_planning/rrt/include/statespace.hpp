#ifndef STATESPACE_HPP
#define STATESPACE_HPP

#include <type_traits>
#include <vector>
#include <functional>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Eigenvalues>

struct Factorial {
  std::vector<int> cache;
  Factorial() { cache.push_back(1); }
  int operator()(int v) {
    if(v < cache.size())
      return cache[v];
    else {
      while(v >= cache.size())
        cache.push_back(cache.back() * (cache.size()));
      return cache.back();
    }
  }
};

template <typename Type, int n>
struct JordanExp
{
  JordanExp() {}
  JordanExp(Eigen::Matrix<Type,n,n> *D) : D(D) {}
  Eigen::Matrix<Type,n,n>
  operator()(Type t) const
  {
    auto tmp = *D;
    Factorial factorial;
    int init_block = 0;
    for(int i=0; i<n; i++) {
      auto d = (*D)(i,i);
      tmp(i,i) = std::exp(d*t);
      for(int k=1; k<=(i-init_block); k++)
        tmp(i-k,i) = (std::pow(t,k)/factorial(k)) * tmp(i,i);
      init_block = (i<n-1 ? ( (((*D)(i+1,i+1)==d) && ((*D)(i,i+1)==1)) ? init_block : i+1 ) : init_block);
    }
    return tmp;
  }
  Eigen::Matrix<Type,n,n> *D = nullptr;
};

template <typename Type, int n, int p, int q, typename ExpFN = JordanExp<Type,n>>
class StateSpace
{
public:
  typedef Eigen::Matrix<Type,n,1> StateType;
  typedef Eigen::Matrix<Type,p,1> InputType;
  typedef Eigen::Matrix<Type,p,1> InputWeightType;
  typedef Eigen::Matrix<Type, n, n> SystemMatrix;
  typedef Eigen::Matrix<Type, n, p> InputMatrix;
  typedef Eigen::Matrix<Type, q, n> OutputMatrix;
  typedef std::function<std::tuple<SystemMatrix,SystemMatrix>(SystemMatrix)> JordanFormHelper;
public:
  StateSpace(SystemMatrix A = SystemMatrix::Identity(),
             InputMatrix B = InputMatrix::Identity(),
             OutputMatrix C = OutputMatrix::Identity(),
             JordanFormHelper fn = nullptr) :
    A(A), B(B), C(C), jordan_form_fn(fn)
  {
    computeTransitionMatrix();
  }

  SystemMatrix expm(Type t) const
  {
    return exp_fn(t);
  }

  void computeTransitionMatrix()
  {
    Eigen::EigenSolver<SystemMatrix> eigen_solver(A);

    // Diagonalization
    // take only real parts of the given complex vectors from eigen, therefore
    // it is your job to ensure that the given system has only real eigen values
    // LIMITATIONS : you should make sure that the resulting Matrix P is Invertible
    // and has FULL RANK, otherwise the computation wouldn't be make sense
    // Hint : just print P_inv to see if it is make sense and balanced enough
    auto eigen_values = eigen_solver.eigenvalues().real();

    // create dictionary for eigen value, and find algebraic multiplicity
    std::vector<std::pair<Type,int>> eig_values;
    std::stringstream eg_dict_ss;
    for(size_t i=0; i<n; i++) {
      auto it = eig_values.begin();
      const auto &e = eigen_values(i);
      for(; it != eig_values.end(); it++) {
        if(std::get<0>(*it) == e)
          break;
      }
      if(it == eig_values.end())
        eig_values.push_back(std::make_pair(e,1));
      else
        std::get<1>(*it) += 1;
    }
    for(const auto &d : eig_values)
      eg_dict_ss << std::get<0>(d) << ": "<< std::get<1>(d) << ", ";

    if((eig_values.size()<n) && (jordan_form_fn)) {
      auto t = jordan_form_fn(A);
      auto J = std::get<0>(t);
      P = std::get<1>(t);
      P_inv = P.inverse();
      D = J;
      //      SystemMatrix a = P*J*P_inv;
    }
    else {
      P = eigen_solver.eigenvectors().real();
      P_inv = P.inverse();
      D = P_inv * A * P;
    }
  }

public:
  SystemMatrix  A;
  InputMatrix   B;
  OutputMatrix  C;
  SystemMatrix P;
  SystemMatrix P_inv;
  SystemMatrix D;
  ExpFN exp_fn;
  JordanFormHelper jordan_form_fn = nullptr;
};

template <typename Type, int n, int p, int q>
class StateSpace<Type,n,p,q,JordanExp<Type,n>>
{
public:
  typedef Eigen::Matrix<Type,n,1> StateType;
  typedef Eigen::Matrix<Type,p,1> InputType;
  typedef Eigen::Matrix<Type,p,p> InputWeightType;
  typedef Eigen::Matrix<Type, n, n> SystemMatrix;
  typedef Eigen::Matrix<Type, n, p> InputMatrix;
  typedef Eigen::Matrix<Type, q, n> OutputMatrix;
  typedef std::function<std::tuple<SystemMatrix,SystemMatrix>(SystemMatrix)> JordanFormHelper;
public:
  StateSpace(SystemMatrix A = SystemMatrix::Identity(),
             InputMatrix B = InputMatrix::Identity(),
             OutputMatrix C = OutputMatrix::Identity(),
             JordanFormHelper fn = nullptr) :
    A(A), B(B), C(C), jordan_form_fn(fn)
  {
    exp_fn = JordanExp<Type,n>(&D);
    computeTransitionMatrix();
  }

  SystemMatrix expm(Type t) const
  {
    return P*exp_fn(t)*P_inv;
  }

  void computeTransitionMatrix()
  {
    Eigen::EigenSolver<SystemMatrix> eigen_solver(A);
    auto eigen_values = eigen_solver.eigenvalues().real();
    std::vector<std::pair<Type,int>> eig_values;
    std::stringstream eg_dict_ss;
    for(size_t i=0; i<n; i++) {
      auto it = eig_values.begin();
      const auto &e = eigen_values(i);
      for(; it != eig_values.end(); it++) {
        if(std::get<0>(*it) == e)
          break;
      }
      if(it == eig_values.end())
        eig_values.push_back(std::make_pair(e,1));
      else
        std::get<1>(*it) += 1;
    }
    for(const auto &d : eig_values)
      eg_dict_ss << std::get<0>(d) << ": "<< std::get<1>(d) << ", ";

    if((eig_values.size()<n) && (jordan_form_fn)) {
      auto t = jordan_form_fn(A);
      auto J = std::get<0>(t);
      P = std::get<1>(t);
      P_inv = P.inverse();
      D = J;
    }
    else {
      P = eigen_solver.eigenvectors().real();
      P_inv = P.inverse();
      D = P_inv * A * P;
    }
  }

public:
  SystemMatrix  A;
  InputMatrix   B;
  OutputMatrix  C;
  SystemMatrix P;
  SystemMatrix P_inv;
  SystemMatrix D;
  JordanExp<Type,n> exp_fn;
  JordanFormHelper jordan_form_fn = nullptr;
};

//template <typename Type, int n, int p, int q>
//Eigen::Matrix<Type,n,n> StateSpace<Type,n,p,q,JordanExp<Type,n> >::expm(Type t) const
//{
//  return P*exp_fn(D,t)*P_inv;

  //    Factorial factorial;
  //    SystemMatrix tmp = D;
  //    int init_block = 0;
  //    for(int i=0; i<n; i++) {
  //      auto d = D(i,i);
  //      tmp(i,i) = std::exp(d*t);
  //      for(int k=1; k<=(i-init_block); k++)
  //        tmp(i-k,i) = (std::pow(t,k)/factorial(k)) * tmp(i,i);
  //      init_block = (i<n-1 ? ( (D(i+1,i+1)==d && D(i,i+1)==1) ? init_block : i+1 ) : init_block);
  //    }
  //    return P*tmp*P_inv;
//}

#endif // STATESPACE_HPP
