#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <random>
#include <cmath>
#include <array>
#include <tuple>
#include "collision.hpp"
#include "random.hpp"

#ifdef GPU
#include <cuda.h>
#include <cuda_runtime.h>
#define HOST __host__
#define DEVICE __device__
#define ATTRIBUTE __host__ __device__
#ifndef gpuErrchk
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess)
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}
#endif
#else
#define ATTRIBUTE
#define DEVICE
#define HOST
#endif

#define DEFAULT_ROBOT_RADIUS (0.26)
#define DEFAULT_SAFETY_RADIUS (0.15)
#define SAMPLE_X0 (11.0)
#define SAMPLE_X1 (7.0)
#define SAMPLE_X2 (1.5)
#define SAMPLE_X3 (1.5)

#ifdef __NVCC__
template <typename scalar> struct Obstacle
{
#else
template <typename scalar>
struct Obstacle : public std::tuple<scalar,scalar>
{
#endif

  ATTRIBUTE
  Obstacle() {}

  HOST
  Obstacle& operator = (const std::tuple<scalar,scalar> rv) {
    (*this)[0] = std::get<0>(rv);
    (*this)[1] = std::get<1>(rv);
    return *this;
  }

  ATTRIBUTE
  inline
  const scalar& operator[](size_t i) const {
#ifdef __NVCC__
    return (i == 0 ? x : (i == 1 ? y : 0));
#else
    switch (i) {
    case 0:
      return std::get<0>(*this);
    case 1:
      return std::get<1>(*this);
    }
#endif
  }

  ATTRIBUTE
  inline
  scalar& operator[](size_t i) {
#ifdef __NVCC__
    return (i == 0 ? x : y);
#else
    switch (i) {
    case 0:
      return std::get<0>(*this);
    case 1:
      return std::get<1>(*this);
    }
#endif
  }

  ATTRIBUTE
  inline
  const scalar& operator ()(size_t i) const {
    return (*this)[i];
  }
#ifdef __NVCC__
  scalar x;
  scalar y;
#endif
};

template <typename scalar = double, int n = 9>
struct Obstacles : public std::array<Obstacle<scalar>,n>
{
  Obstacles() {}

  inline
  std::array<std::tuple<scalar,scalar>,n> operator()
  (std::array<std::tuple<scalar,scalar>,n> &lhs, Obstacles &rhs) {
    for(size_t i=0; i<n; i++)
      lhs[i] = std::tuple<scalar,scalar>(rhs[i]);
    return lhs;
  }
};

#ifdef __NVCC__
template <typename scalar>
struct DynamicObstacle
{
#else
template <typename scalar>
// struct DynamicObstacle : public std::tuple<scalar,scalar,scalar,scalar>
struct DynamicObstacle : public std::array<scalar,4>
{
#endif
  ATTRIBUTE
  DynamicObstacle() {}

  HOST
  DynamicObstacle& operator = (const std::array<scalar,4> &rv) {
    (*this)[0] = std::get<0>(rv);
    (*this)[1] = std::get<1>(rv);
    (*this)[2] = std::get<2>(rv);
    (*this)[3] = std::get<3>(rv);
    return *this;
  }

  template <typename time>
  ATTRIBUTE
  inline
  DynamicObstacle<scalar>
  operator ()(time t) const {
    DynamicObstacle<scalar> ret;
    ret[0] = (*this)[0] + (*this)[2] * t;
    ret[1] = (*this)[1] + (*this)[3] * t;
    return ret;
  }

  // template <>
  // gcc doesn't allow inline explicit template specialization, btw
  // but this does the trick
  ATTRIBUTE
  inline
  scalar operator ()(size_t i) const {
    return (*this)[i];
  }
  ATTRIBUTE
  inline
  scalar operator ()(int i) const {
    return (*this)[i];
  }

#ifdef __NVCC__
  __device__
  inline
  scalar& operator[](size_t i) {
    return state[i];
  }
  __device__
  inline
  const scalar& operator[](size_t i) const {
    return state[i];
  }
  scalar state[4];
#endif
};

template <typename scalar = double, int n = 9>
struct DynamicObstacles : public std::array<DynamicObstacle<scalar>,n>
{
  DynamicObstacles() {}

  inline
  std::array<std::tuple<scalar,scalar,scalar,scalar>,n> operator()
  (std::array<std::tuple<scalar,scalar,scalar,scalar>,n> &lhs, DynamicObstacles &rhs) {
    for(size_t i=0; i<n; i++)
      lhs[i] = std::tuple<scalar,scalar,scalar,scalar>(rhs[i]);
    return lhs;
  }
};

template <typename scalar = double, int n = 9>
struct Robosoccer
{
  Robosoccer(scalar x0 = scalar(SAMPLE_X0), scalar x1 = scalar(SAMPLE_X1))
  {
    rg = new RandomGen<2,scalar>({-x0,-x1},{x0,x1});
    for(auto &o : obs)
      o = std::make_tuple(scalar(0.0),scalar(0.0));
  }
  Robosoccer(const std::array<std::tuple<scalar,scalar>,n> &obs,
             scalar x0 = scalar(SAMPLE_X0),
             scalar x1 = scalar(SAMPLE_X1))
  {
    auto i=0;
    for(const auto &o : obs)
      this->obs.at(i++) = o;
    rg = new RandomGen<2,scalar>({-x0,-x1},{x0,x1});
  }

#ifndef __NVCC__
  void setRandomObstacles() {
    for(auto& o : obs) {
      std::get<0>(o) = (*rg)(0);
      std::get<1>(o) = (*rg)(1);
    }
  }
#endif

  template<typename ArrayLike, size_t x_idx=0, size_t y_idx=1>
  void setObstacles(const ArrayLike &obstacles) {
    for(size_t i=0; i<obstacles.size(); i++) {
      if(i == obs.size()) break;
      std::get<0>(obs.at(i)) = obstacles[i](x_idx);
      std::get<1>(obs.at(i)) = obstacles[i](y_idx);
    }
  }

  template <int x_idx=0, int y_idx=1, typename point_t>
  inline
  bool collide(const point_t &pt) const {
    return point_circle_collision<x_idx,y_idx>(pt,this->obs,this->collision_radius);
  }

  // see http://paulbourke.net/geometry/pointlineplane/
  template <int x_idx=0, int y_idx=1, typename p1_t,  typename p2_t>
  inline
  bool collide(const p1_t &pt0, const p2_t &pt1) const {
    return line_circle_collision<x_idx,y_idx>(pt0,pt1,this->obs,this->collision_radius);
  }

  // std::array<Obstacle<scalar>,n> obs;
  Obstacles<scalar,n> obs;
  scalar safety_radius = DEFAULT_SAFETY_RADIUS;
  scalar single_robot_radius = DEFAULT_ROBOT_RADIUS;
  scalar collision_radius = (DEFAULT_ROBOT_RADIUS+DEFAULT_SAFETY_RADIUS)*2;
  RandomGen<2,scalar> *rg;
};

template <typename scalar = double, int n = 9>
struct DynamicRobosoccer
{
  DynamicRobosoccer(scalar x0 = scalar(SAMPLE_X0),
                    scalar x1 = scalar(SAMPLE_X1),
                    scalar x2 = scalar(SAMPLE_X2),
                    scalar x3 = scalar(SAMPLE_X3))
  {
    rg = new RandomGen<4>({-x0,-x1,-x2,-x3},{x0,x1,x2,x3});
    for(auto &o : obs)
      o = DynamicObstacle<scalar>();
  }
  DynamicRobosoccer(const std::array<DynamicObstacle<scalar>,n> &obs,
                    scalar x0 = scalar(SAMPLE_X0),
                    scalar x1 = scalar(SAMPLE_X1),
                    scalar x2 = scalar(SAMPLE_X2),
                    scalar x3 = scalar(SAMPLE_X3))
    : obs(obs)
  {
    rg = new RandomGen<4,scalar>({-x0,-x1,-x2,-x3},{x0,x1,x2,x3});
  }

  inline
  std::array<DynamicObstacle<scalar>,n>
  at (scalar time) {
    std::array<DynamicObstacle<scalar>,n> ret;
    for(size_t i=0; i<ret.size(); i++)
      ret[i] = obs[i](time);
    return ret;
  }

  void setRandomObstacles() {
    for(auto& o : obs) {
#ifndef __NVCC__
      std::get<0>(o) = (*rg)(0);
      std::get<1>(o) = (*rg)(1);
      std::get<2>(o) = (*rg)(2);
      std::get<3>(o) = (*rg)(3);
#else
      (*rg)(o.state);
#endif
    }
  }

  template<typename ArrayLike, size_t x_idx=0, size_t y_idx=1, size_t vx_idx=2, size_t vy_idx=3>
  void setObstacles(const ArrayLike &obstacles) {
    for(size_t i=0; i<obstacles.size(); i++) {
      if(i == obs.size()) break;
      std::get<0>(obs.at(i)) = obstacles[i](x_idx);
      std::get<1>(obs.at(i)) = obstacles[i](y_idx);
      std::get<2>(obs.at(i)) = obstacles[i](vx_idx);
      std::get<3>(obs.at(i)) = obstacles[i](vy_idx);
    }
  }

  template <int x_idx=0, int y_idx=1, typename point_t>
  bool collide(const point_t &pt) const {
    return point_circle_collision<x_idx,y_idx>(pt,this->obs,this->collision_radius);
  }

  // see http://paulbourke.net/geometry/pointlineplane/
  template <int x_idx=0, int y_idx=1, int segment = 10, typename p1_t,  typename p2_t>
  bool collide(const p1_t &pt0, const p2_t &pt1, const scalar &t0, const scalar &t1) const {
    return parametrized_line_circle_collision<x_idx,y_idx,segment>(pt0,pt1,t0,t1,obs,collision_radius);
  }

  // std::array<DynamicObstacle<scalar>,n> obs;
  DynamicObstacles<scalar,n> obs;
  const scalar safety_radius = DEFAULT_SAFETY_RADIUS;
  const scalar single_robot_radius = DEFAULT_ROBOT_RADIUS;
  const scalar collision_radius = (DEFAULT_ROBOT_RADIUS+DEFAULT_SAFETY_RADIUS)*2;
  RandomGen<4,scalar> *rg;
};

#endif // ENVIRONMENT_HPP
