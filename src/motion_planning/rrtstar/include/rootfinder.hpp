#ifndef ROOTFINDER_HPP
#define ROOTFINDER_HPP

#include <cmath>

#ifdef GPU
#include <cuda.h>
#include <cuda_runtime.h>
#define ATTRIBUTE __host__ __device__
#else
#define ATTRIBUTE
#endif

namespace RootFinder {

template
<
    // an FN class should overloads operator () with (numeric)->numeric signature
    class FN
>
// find root of a function using secant method
class SecantMethod
{
public:
  ATTRIBUTE SecantMethod(const FN& f)
  : f(f) {}
  template<typename scalar, typename err_t, int MaxIter=1000>
  ATTRIBUTE scalar operator()(scalar x1, scalar x0, err_t e) const
  {
    auto f0 = f(x0);
    auto f1 = f(x1);
    auto xr = x1 - (f1 * (x0 - x1)/(f0 - f1));
    auto xr_old = xr;
    err_t ea = e;
    for(int i=0; i<MaxIter; i++) {
      xr_old = xr;
      f0 = f1; x0 = x1; x1 = xr;
      f1 = f(x1);
      xr = x1 - (f1 * (x0 - x1)/(f0 - f1));
      ea = std::abs(f1);
      if(ea < e)
        break;
    }
    return xr;
  }
public:
  const FN& f;
};

}

#endif // ROOTFINDER_HPP
