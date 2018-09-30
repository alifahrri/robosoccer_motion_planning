#ifndef INTEGRATOR_HPP
#define INTEGRATOR_HPP

#include <cmath>

template <class T, class F, int max_depth>
struct SimpsonsRuleIntegrator
{
  SimpsonsRuleIntegrator(const F &f, const T& eps)
    : f(f), eps(eps) {}

  const F &f;
  const T eps;
  int depth = 0;

  T operator()(const T &a, const T &b) const {
    T c = (a + b) / 2; T h = b - a;
    T fa = f(a); T fb = f(b); T fc = f(c);
    T s = (h/6) * (fa + 4*fc + fb);
    return aux_opt(a,b,s,fa,fb,fc);
  }

  T aux_opt(const T& a, const T &b, const T& S,
            const T& fa, const T& fb, const T& fc) const {
    T c = (a + b) / 2;
    T h = (b - a);
    T d = (a + c) / 2;
    T e = (c + b) / 2;
    T fd = f(d); T fe = f(e);
    //    T fa = f(a); T fb = f(b); T fc = f(c);
    T s_left = (h/12) * (fa + 4*fd + fc);
    T s_right = (h/12) * (fc + 4*fe + fb);
    T s = s_left + s_right;
    if((this->depth >= max_depth) || (fabs(S-s) <= (15*eps)))
      return s + (s-S)/15;
    else {
      SimpsonsRuleIntegrator f_aux_left(f,this->eps/2);
      SimpsonsRuleIntegrator f_aux_right(f,this->eps/2);
      f_aux_left.depth = f_aux_right.depth = this->depth+1;
      return f_aux_left.aux_opt(a, c, s_left, fa, fc, fd) +
          f_aux_right.aux_opt(c, b, s_right, fc, fb, fe);
    }
  }
};

#endif // INTEGRATOR_HPP
