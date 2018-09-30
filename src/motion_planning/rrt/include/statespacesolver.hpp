#ifndef STATESPACESOLVER_HPP
#define STATESPACESOLVER_HPP

#include <iostream>
#include <vector>
#include <functional>
#include "statespace.hpp"
#include "integrator.hpp"

template <typename Type, int n, int p, int q, typename SSType>
class StateSpaceSolver
{
public:
  // typedef StateSpace<Type,n,p,q> SSType;
  typedef Eigen::Matrix<Type,p,1> Input;
  typedef Eigen::Matrix<Type,n,1> State;
  typedef Eigen::Matrix<Type,n,n> System;
  typedef std::vector<Input> InputsType;
  typedef std::vector<State> StatesType;
  typedef std::vector<Type> TimeArray;

  //  typedef Eigen::Matrix<Type,n,n> MatType;
  using MatType = typename SSType::SystemMatrix;

  typedef std::function<std::tuple<MatType,MatType>(MatType)> JordanFormHelper;

private:
  // This is a helper for evaluating integral of input response part
  struct FunctionAux {
    FunctionAux(const StateSpaceSolver &solver)
      : solver(solver) {}
    const StateSpaceSolver &solver;
    int idx = 0;
    InputsType u;
    Type T = 0;
    Type dt = 0;
    Type operator()(const Type &t) const {
      const auto &B = solver.system.B;
      auto u_idx = std::min((size_t)(t/dt),u.size()-1);
      auto eAt_idx = solver.system.expm(T-t).row(idx);
      return eAt_idx*B*u[u_idx];
    }
  };

public:
  StateSpaceSolver(SSType &system)
    : system(system), ir_fn(*this) {}
  State solve(Type t, Input u)  const;
  State solve(Type t, State x0) const;
  State solve(Type t, State x0, Input u) const;
  State solve(Type t, InputsType u) const;
  StatesType solve(TimeArray time_array, State x0) const;
  StatesType solve(TimeArray time_array, InputsType u) const;
  StatesType solve(TimeArray time_array, State x0, InputsType u) const;

public:
  const SSType &system;
  FunctionAux ir_fn;

private:
  // comput input response, assuming equally spaced inputs vector
  State input_response(Type t, InputsType u_array) const {
    State res;
    res = State::Zero();
    FunctionAux ir_fn(*this);
    SimpsonsRuleIntegrator<Type,FunctionAux,15> integrator(ir_fn,(Type)0.1f);
    ir_fn.u = u_array;
    ir_fn.T = t;
    ir_fn.dt = t/u_array.size();
    for(int idx=0; idx<n; idx++) {
      ir_fn.idx = idx;
      auto sum = integrator((Type)0.0,t);
      res(idx) = sum;
    }

    return res;
  }

  State initial_response(Type t, State x0) const
  {
    State ret;
    ret = State::Zero();
    auto eAt = system.expm(t);
    ret = eAt * x0;
    return ret;
  }
};

template<typename Type, int n, int p, int q, typename SSType>
typename
StateSpaceSolver<Type,n,p,q,SSType>::State StateSpaceSolver<Type,n,p,q,SSType>::solve(Type t, typename StateSpaceSolver<Type,n,p,q,SSType>::Input u) const
{
  InputsType u_array;
  u_array.push_back(u);
  return input_response(t, u_array);
}

template<typename Type, int n, int p, int q, typename SSType>
typename
StateSpaceSolver<Type,n,p,q,SSType>::State StateSpaceSolver<Type,n,p,q,SSType>::solve(Type t, StateSpaceSolver::InputsType u) const
{
  return input_response(t, u);
}

template<typename Type, int n, int p, int q, typename SSType>
typename
StateSpaceSolver<Type,n,p,q,SSType>::State StateSpaceSolver<Type,n,p,q,SSType>::solve(Type t, typename StateSpaceSolver<Type,n,p,q,SSType>::State x0) const
{
  return initial_response(t, x0);
}

template<typename Type, int n, int p, int q, typename SSType>
typename
StateSpaceSolver<Type,n,p,q,SSType>::State StateSpaceSolver<Type,n,p,q,SSType>::solve(Type t, typename StateSpaceSolver<Type,n,p,q,SSType>::State x0, StateSpaceSolver::Input u) const
{
  InputsType u_array;
  u_array.push_back(u);
  return initial_response(t, x0) + input_response(t, u_array);
}

template<typename Type, int n, int p, int q, typename SSType>
typename
StateSpaceSolver<Type,n,p,q,SSType>::StatesType StateSpaceSolver<Type,n,p,q,SSType>::solve(StateSpaceSolver::TimeArray t, StateSpaceSolver::InputsType u) const
{
  StatesType outs;
  for(const auto& _t : t) {
    outs.push_back(input_response(_t,u));
  }
  return outs;
}

template<typename Type, int n, int p, int q, typename SSType>
typename
StateSpaceSolver<Type,n,p,q,SSType>::StatesType StateSpaceSolver<Type,n,p,q,SSType>::solve(StateSpaceSolver::TimeArray time_array, StateSpaceSolver::State x0) const
{
  StatesType outs;
  for(const auto& t : time_array) {
    outs.push_back(initial_response(t,x0));
  }
  return outs;
}

template<typename Type, int n, int p, int q, typename SSType>
typename
StateSpaceSolver<Type,n,p,q,SSType>::StatesType StateSpaceSolver<Type,n,p,q,SSType>::solve(StateSpaceSolver::TimeArray time_array, StateSpaceSolver::State x0, StateSpaceSolver::InputsType u) const
{
  StatesType result;
  for(size_t i=0; i<time_array.size(); i++) {
    InputsType u_array;
    Type t = time_array[i];
    if(i>0) {
      auto dt = time_array[i]-time_array[i-1];
      t = dt;
    }
    u_array.push_back(u[i]);
    auto init_res = initial_response(t,(i==0?x0:result.back()));
    auto input_res = input_response(t,u_array);
    auto res = init_res + input_res;
    result.push_back(res);
  }
  return result;
}

#endif // STATESPACESOLVER_HPP
