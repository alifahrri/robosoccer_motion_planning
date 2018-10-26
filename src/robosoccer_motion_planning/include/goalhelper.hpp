#ifndef GOALHELPER_HPP
#define GOALHELPER_HPP

#include <type_traits>
#include "elements.hpp"

auto set_xyz(const auto &src, auto &dst)
 -> decltype(x(src), x(dst), y(src), y(dst), z(src), z(dst), void())
{
  x(dst) = (std::decay_t<decltype(x(dst))>) x(src);
  y(dst) = (std::decay_t<decltype(y(dst))>) y(src);
  z(dst) = (std::decay_t<decltype(z(dst))>) z(src);
}

auto set_xyz(auto &src, auto &dst)
 -> decltype(x(src), x(dst), y(src), y(dst), z(src), z(dst), void())
{
  x(dst) = x(src);
  y(dst) = y(src);
  z(dst) = z(src);
}

void get_header(...)
{
  // ignore
}

auto get_header(auto &t) -> decltype(t.header)
{
  return t.header;
}

auto get_header(auto &t) -> decltype(t->header)
{
  return t->header;
}

void get_stamp(...)
{
  // ignore
}

auto get_stamp(auto &t) -> decltype(t.stamp)
{
  return t.stamp;
}

auto get_stamp(auto &t) -> decltype(t->stamp)
{
  return t->stamp;
}

auto get_stamp(const auto &t) -> decltype(t.stamp)
{
  return t.stamp;
}

auto get_stamp(const auto &t) -> decltype(t->stamp)
{
  return t->stamp;
}

auto get_position(auto &v) -> decltype(v.position)
{
  return v.position;
}

auto get_position(const auto &v) -> decltype(v.position)
{
  return v.position;
}

auto get_position(auto &v) -> decltype(v->position)
{
  return v->position;
}

auto get_position(const auto &v) -> decltype(v->position)
{
  return v->position;
}

auto get_position(auto &v)
{
  // called if v doesn't have member named position
  return v;
}

auto get_position(...)
{

}

auto get_pose(auto &v) -> decltype(v.pose)
{
  return v.pose;
}

auto get_pose(auto &v) -> decltype(v->pose)
{
  return v->pose;
}

auto get_pose(...)
{
  // sink hole for position
}

#endif // GOALHELPER_HPP
