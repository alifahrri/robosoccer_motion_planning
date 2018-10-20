#ifndef COLLISION_HPP
#define COLLISION_HPP

#include <vector>
#include <math.h>
#include <type_traits>

#ifdef GPU
#include <cuda.h>
#include <cuda_runtime.h>
#include "util.cuh"

#define HOST __host__
#define DEVICE __device__
#define ATTRIBUTE __host__ __device__
#else
#define HOST
#define DEVICE
#define ATTRIBUTE
#endif

#ifndef NDEBUG
#ifdef __NVCC__
#include "util.h"
#define TRACE_CUDA
#endif
#endif

template <int x_idx, int y_idx, typename point_t, typename Iterable, typename scalar>
ATTRIBUTE
inline
bool point_circle_collision(const point_t &pt, const Iterable &obs, const scalar &collision_radius) {
  bool collision = false;
  // for(size_t i=0; i<obs.size(); i++)
  for(const auto &o : obs) {
    // auto dx = pt(x_idx) - std::get<0>(o);
    // auto dy = pt(y_idx) - std::get<1>(o);
    auto dx = pt(x_idx) - o(x_idx);
    auto dy = pt(y_idx) - o(y_idx);
    auto r = sqrt(dx*dx+dy*dy);
    if(r <= collision_radius) {
      collision = true;
      break;
    }
  }
  return collision;
}

// see http://paulbourke.net/geometry/pointlineplane/
template <int x_idx, int y_idx, typename p1_t, typename p2_t, typename Iterable, typename scalar>
ATTRIBUTE
inline
bool line_circle_collision(const p1_t &pt0, const p2_t &pt1, const Iterable &obs, const scalar &collision_radius) {
#ifdef TRACE_CUDA
  TRACE_KERNEL(blockIdx.x * blockDim.x + threadIdx.x, 0, __PRETTY_FUNCTION__);
#endif
  scalar x0, x1, y0, y1;
  bool collision = false;
  x0 = pt0(x_idx); y0 = pt0(y_idx);
  x1 = pt1(x_idx); y1 = pt1(y_idx);
  if(isnan(x0) || isnan(x1) || isnan(y0) || isnan(y1))
    return true;
  //  for(size_t i=0; i<obs.size(); i++)
  for(const auto &o : obs) {
    // auto cx = std::get<0>(o);
    // auto cy = std::get<1>(o);
    auto cx = o(x_idx);
    auto cy = o(y_idx);
    // naive collision check
    auto dxc0 = pt0(x_idx) - cx; auto dxc1 = pt1(x_idx) - cx;
    auto dyc0 = pt0(y_idx) - cy; auto dyc1 = pt1(y_idx) - cy;
    auto r = sqrt(dxc0*dxc0+dyc0*dyc0);
    if(r <= collision_radius) {
      collision = true;
      break;
    }
    r = sqrt(dxc1*dxc1+dyc1*dyc1);
    if(r <= collision_radius) {
      collision = true;
      break;
    }
    auto dx = (x1-x0); auto dy = (y1-y0);
    auto u = ((cx-x0)*(x1-x0) + (cy-y0)*(y1-y0)) / (dx*dx+dy*dy);
    if(u<scalar(0.0) || u>scalar(1.0))
      continue;
    auto x = x0+u*(dx); auto y = y0+u*(dy);
    auto dxr = x-cx; auto dyr = y-cy;
    r = sqrt(dxr*dxr+dyr*dyr);
    if(r <= collision_radius) {
      collision = true;
      break;
    }
  }
#ifdef TRACE_CUDA
  TRACE_KERNEL(blockIdx.x * blockDim.x + threadIdx.x, 0, __FUNCTION__, "OK");
#endif
  return collision;
}

template <int x_idx=0, int y_idx=1, int segment = 10, typename p1_t, typename p2_t, typename scalar, typename ArrayLike>
inline
bool parametrized_line_circle_collision(const p1_t &p1, const p2_t &p2, const scalar &t0, const scalar &t1, const ArrayLike &obs, const scalar &collision_radius)
{
  auto collision = false;
  scalar dt = (t1-t0)/segment;
  auto dpx = p2(x_idx) - p1(x_idx);
  auto dpy = p2(y_idx) - p1(y_idx);
  auto p1_test = p1;
  auto p2_test = p1;
  p1_test(x_idx) = p1(x_idx);
  p1_test(y_idx) = p1(y_idx);
  using ObsType = std::decay_t<decltype(obs[0])>;
  // auto obs_t0 = obs;
  // auto obs_t1 = obs;
  std::vector<ObsType> obs_t0, obs_t1;
  obs_t0.resize(obs.size());
  obs_t1.resize(obs.size());
  for(size_t i=1; i<=segment; i++) {
    scalar ti = t0+(i-1)*dt;
    scalar tf = ti+dt;
    for(size_t j=0; j<obs.size(); j++) {
      obs_t0[j] = obs[j](ti);
      obs_t1[j] = obs[j](tf);
    }
    p2_test(x_idx) = p1(x_idx) + (scalar(i) / segment * dpx);
    p2_test(y_idx) = p1(y_idx) + (scalar(i) / segment * dpy);
    if(line_circle_collision<x_idx,y_idx>(p1_test, p2_test, obs_t0, collision_radius) ||
       line_circle_collision<x_idx,y_idx>(p1_test, p2_test, obs_t1, collision_radius) ) {
      collision = true;
      break;
    }
    p1_test = p2_test;
  }
  return collision;
}

#endif // COLLISION_HPP

