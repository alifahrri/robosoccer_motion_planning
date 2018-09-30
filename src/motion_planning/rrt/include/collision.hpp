#ifndef COLLISION_HPP
#define COLLISION_HPP

#include <cmath>

template <int x_idx, int y_idx, typename point_t, typename Iterable, typename scalar>
inline
bool point_circle_collision(const point_t &pt, const Iterable &obs, const scalar &collision_radius) {
  bool collision = false;
  for(const auto &o : obs) {
    // auto dx = pt(x_idx) - std::get<0>(o);
    // auto dy = pt(y_idx) - std::get<1>(o);
    auto dx = pt(x_idx) - o(x_idx);
    auto dy = pt(y_idx) - o(y_idx);
    auto r = std::sqrt(dx*dx+dy*dy);
    if(r <= collision_radius) {
      collision = true;
      break;
    }
  }
  return collision;
}

// see http://paulbourke.net/geometry/pointlineplane/
template <int x_idx, int y_idx, typename p1_t, typename p2_t, typename Iterable, typename scalar, int segment = 10>
inline
bool line_circle_collision(const p1_t &pt0, const p2_t &pt1, const Iterable &obs, const scalar &collision_radius) {
  scalar x0, x1, y0, y1;
  bool collision = false;
  x0 = pt0(x_idx); y0 = pt0(y_idx);
  x1 = pt1(x_idx); y1 = pt1(y_idx);
  if(std::isnan(x0) || std::isnan(x1) || std::isnan(y0) || std::isnan(y1))
    return true;
  for(const auto &o : obs) {
    // auto cx = std::get<0>(o);
    // auto cy = std::get<1>(o);
    auto cx = o(x_idx);
    auto cy = o(y_idx);
    // naive collision check
    auto dxc0 = pt0(x_idx) - cx; auto dxc1 = pt1(x_idx) - cx;
    auto dyc0 = pt0(y_idx) - cy; auto dyc1 = pt1(y_idx) - cy;
    auto r = std::sqrt(dxc0*dxc0+dyc0*dyc0);
    if(r <= collision_radius) {
      collision = true;
      break;
    }
    r = std::sqrt(dxc1*dxc1+dyc1*dyc1);
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
    r = std::sqrt(dxr*dxr+dyr*dyr);
    if(r <= collision_radius) {
      collision = true;
      break;
    }
  }
  return collision;
}

template <int x_idx, int y_idx, int vx_idx, int vy_idx, typename p1_t, typename p2_t, typename scalar, typename Iterable, int segment = 10>
bool time_parametrized_line_circle_collision(const p1_t &p1, const p2_t &p2, const scalar &t0, const scalar &t1, const Iterable &obs, const scalar &collision_radius)
{
  return false;
}

#endif // COLLISION_HPP
