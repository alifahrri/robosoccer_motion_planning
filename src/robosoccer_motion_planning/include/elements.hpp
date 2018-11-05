#ifndef ELEMENTS_HPP
#define ELEMENTS_HPP

// no, this doesn't have postulate number five
// just generic helper functions to set or get xyz member

// provide overload resolution for getting positon

// vy :
auto y(auto &p) -> decltype((p.y))
{
  return p.y;
}

auto y(auto &p, int i=1) -> decltype((p[i]))
{
  return p[i];
}

// vx :
auto x(auto &p) -> decltype((p.x))
{
  return p.x;
}

auto x(auto &p, int i=0) -> decltype((p[i]))
{
  return p[i];
}

// vz :
auto z(auto &p) -> decltype((p.z))
{
  return (p.z);
}

auto z(auto &p, int i=2) -> decltype(p[i])
{
  return p[i];
}

// provide overload resolution for getting velocity

// vx :
auto vx(auto &p) -> decltype((p.vx))
{
  return p.vx;
}

auto vx(auto &p) -> decltype(p.vel, (x(p.vel)))
{
  return x(p.vel);
}

auto vx(auto &p, int i=2) -> decltype((p[i]))
{
  return p[i];
}

// vy :
auto vy(auto &p) -> decltype((p.vy))
{
  return p.vy;
}

auto vy(auto &p) -> decltype(p.vel, (y(p.vel)))
{
  return y(p.vel);
}

auto vy(auto &p, int i=3) -> decltype((p[i]))
{
  return p[i];
}

// vz :
auto vz(auto &p) -> decltype((p.vz))
{
  return (p.vz);
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
