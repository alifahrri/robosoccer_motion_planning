#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <random>
#include <cmath>
#include <array>
#include <tuple>
#include "collision.hpp"
#include "random.hpp"

#define DEFAULT_ROBOT_RADIUS (0.26)
#define DEFAULT_SAFETY_RADIUS (0.15)
#define SAMPLE_X0 (11.0)
#define SAMPLE_X1 (7.0)
#define SAMPLE_X2 (1.5)
#define SAMPLE_X3 (1.5)

template <typename scalar>
struct Obstacle : public std::tuple<scalar,scalar>
{
  Obstacle() {}

  Obstacle& operator = (const std::tuple<scalar,scalar> rv) {
    std::get<0>(*this) = std::get<0>(rv);
    std::get<1>(*this) = std::get<1>(rv);
    return *this;
  }

  inline
  const scalar& operator[](size_t i) const {
    switch (i) {
    case 0:
      return std::get<0>(*this);
    case 1:
      return std::get<1>(*this);
    }
  }

  inline
  scalar& operator[](size_t i) {
    switch (i) {
    case 0:
      return std::get<0>(*this);
    case 1:
      return std::get<1>(*this);
    }
  }

  inline
  const scalar& operator ()(size_t i) const {
    switch (i) {
    case 0:
      return std::get<0>(*this);
    case 1:
      return std::get<1>(*this);
    default:
      return scalar(0);
    }
  }
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

template <typename scalar>
struct DynamicObstacle : public std::tuple<scalar,scalar,scalar,scalar>
{
  DynamicObstacle() {}

  DynamicObstacle& operator = (const std::tuple<scalar,scalar,scalar,scalar> rv) {
    std::get<0>(*this) = std::get<0>(rv);
    std::get<1>(*this) = std::get<1>(rv);
    std::get<2>(*this) = std::get<2>(rv);
    std::get<3>(*this) = std::get<3>(rv);
    return *this;
  }

  inline
  const scalar& operator[](size_t i) const {
    switch (i) {
    case 0:
      return std::get<0>(*this);
    case 1:
      return std::get<1>(*this);
    case 2:
      return std::get<2>(*this);
    case 3:
      return std::get<3>(*this);
    }
  }

  inline
  scalar& operator[](size_t i) {
    switch (i) {
    case 0:
      return std::get<0>(*this);
    case 1:
      return std::get<1>(*this);
    case 2:
      return std::get<2>(*this);
    case 3:
      return std::get<3>(*this);
    }
  }

  inline
  std::tuple<scalar,scalar>
  operator ()(scalar t) const {
    std::tuple<scalar,scalar> ret;
    std::get<0>(ret) = std::get<0>(*this) * std::get<2>(*this) * t;
    std::get<1>(ret) = std::get<1>(*this) * std::get<3>(*this) * t;
    return ret;
  }

  inline
  const scalar& operator ()(size_t i) const {
    switch (i) {
    case 0:
      return std::get<0>(*this);
    case 1:
      return std::get<1>(*this);
    case 2:
      return std::get<2>(*this);
    case 3:
      return std::get<3>(*this);
    default:
      return scalar(0);
    }
  }
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
struct Robosoccer {
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

  void setRandomObstacles() {
    for(auto& o : obs) {
      std::get<0>(o) = (*rg)(0);
      std::get<1>(o) = (*rg)(1);
    }
  }

  template<typename ArrayLike, size_t x_idx=0, size_t y_idx=1>
  void setObstacles(const ArrayLike &obstacles) {
    for(size_t i=0; i<obstacles.size(); i++) {
      if(i == obs.size()) break;
      std::get<0>(obs.at(i)) = obstacles[i](x_idx);
      std::get<1>(obs.at(i)) = obstacles[i](y_idx);
    }
  }

  template <typename point_t, int x_idx, int y_idx>
  bool collide(const point_t &pt) const {
    return point_circle_collision<x_idx,y_idx>(pt,this->obs,this->collision_radius);
  }

  // see http://paulbourke.net/geometry/pointlineplane/
  template <int x_idx, int y_idx, typename p1_t,  typename p2_t, int segment = 10>
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
struct DynamicRobosoccer {
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

  void setRandomObstacles() {
    for(auto& o : obs) {
      std::get<0>(o) = *rg(0);
      std::get<1>(o) = *rg(1);
      std::get<2>(o) = *rg(2);
      std::get<3>(o) = *rg(3);
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

  template <typename point_t, int x_idx, int y_idx>
  bool collide(const point_t &pt) const {
    return point_circle_collision<x_idx,y_idx>(pt,this->obs,this->collision_radius);
  }

  // see http://paulbourke.net/geometry/pointlineplane/
  template <int x_idx, int y_idx, int vx_idx, int vy_idx, typename p1_t,  typename p2_t, int segment = 10>
  bool collide(const p1_t &pt0, const p2_t &pt1, const scalar &t0, const scalar &t1) const {
    return time_parametrized_line_circle_collision<x_idx,y_idx,vx_idx,vy_idx>(pt0,pt1,t0,t1,this->obs,this->collision_radius);
  }

  // std::array<DynamicObstacle<scalar>,n> obs;
  DynamicObstacles<scalar,n> obs;
  const scalar safety_radius = DEFAULT_SAFETY_RADIUS;
  const scalar single_robot_radius = DEFAULT_ROBOT_RADIUS;
  const scalar collision_radius = (DEFAULT_ROBOT_RADIUS+DEFAULT_SAFETY_RADIUS)*2;
  RandomGen<4,scalar> *rg;
};

#endif // ENVIRONMENT_HPP
