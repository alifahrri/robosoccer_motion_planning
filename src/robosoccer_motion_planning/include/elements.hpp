#ifndef ELEMENTS_HPP
#define ELEMENTS_HPP

// no, this doesn't have postulate number five
// just generic helper functions to set or get xyz member

auto y(auto &p) -> decltype((p.y))
{
  return p.y;
}

auto x(auto &p) -> decltype((p.x))
{
  return p.x;
}

auto z(auto &p) -> decltype((p.z))
{
  return (p.z);
}

auto y(auto &p, int i=1) -> decltype((p[i]))
{
  return p[i];
}

auto x(auto &p, int i=0) -> decltype((p[i]))
{
  return p[i];
}

auto z(auto &p, int i=2) -> decltype(p[i])
{
  return p[i];
}

auto angle(auto &p, int i=3) -> decltype(p[i])
{
  return p[i];
}

auto angle(auto &p) -> decltype((p.w))
{
  return p.w;
}

#endif // ELEMENTS_HPP
