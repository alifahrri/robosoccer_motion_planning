#ifndef POLYNOMIAL_HPP
#define POLYNOMIAL_HPP

#include <tuple>
#include <cmath>
#include <stdexcept>

// see https://en.wikipedia.org/wiki/Quartic_function#General_formula_for_roots
template
<typename scalar>
std::tuple<scalar,scalar,scalar,scalar>
solve_quartic(scalar a, scalar b, scalar c, scalar d, scalar e)
{
  std::tuple<scalar,scalar,scalar,scalar> ret;
  auto a2 = a*a;
  auto a3 = a2*a;
  auto b2 = b*b;
  auto b3 = b2*b;
  auto c2 = c*c;
  auto c3 = c2*c;
  auto d2 = d*d;
  auto ac = a*c;
  auto p = (8*ac - 3*b2)/(8*a2);
  auto q = (b3-4*ac*b+8*a2*d)/(8*a3);
  auto d0 = c2-3*b*d+12*a*e;
  auto d1 = 2*c3-9*b*c*d+27*b2*e+27*a*d2-72*ac*e;
  auto Q = std::pow((d1+std::sqrt(d1*d1-4*d0*d0*d0)),1.0/3);
  auto S = std::sqrt((-2*p/3)+(1/3*a)*(Q+d0/Q));
  auto b4a = b/(4*a);
  auto tmp = -4*S*S-2*p;
  auto qS = q/S;
  std::get<0>(ret) = -b4a - S + std::sqrt(tmp+qS)/2;
  std::get<1>(ret) = -b4a - S - std::sqrt(tmp+qS)/2;
  std::get<2>(ret) = -b4a + S + std::sqrt(tmp-qS)/2;
  std::get<3>(ret) = -b4a + S - std::sqrt(tmp-qS)/2;
  if(S==0 || Q==0)
    throw std::runtime_error("special case for quartic function not implemented!!");
  return ret;
}

#endif // POLYNOMIAL_HPP
