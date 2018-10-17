#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

#include <vector>
#include <random>
#include <cstdio>

// template <typename T> using vector = std::vector<T>;
using namespace std;

template <typename T, int n>
struct Point
{
  Point() {}
  T p[n];
  const T& operator [](int i) const
  {
    return p[i];
  }
  T& operator [](int i)
  {
    return p[i];
  }
  const T* data() const { return p; }
  const T& operator() (int i) const {
    return p[i];
  }
  T& operator ()(int i) {
    return p[i];
  }
};

template <typename T, int n>
struct PointCloud
{
  PointCloud() {}
  vector<Point<T,n>> states;
  void clear() { states.clear(); }
  inline void push_back(const Point<T,n> &p) { states.push_back(p); }
  inline const Point<T,n>& operator[] (const size_t &i) const { return states[i]; }
  inline Point<T,n>& operator[] (const size_t &i) { return states[i]; }

  inline size_t size() const { return states.size(); }

  inline size_t kdtree_get_point_count() const { return states.size(); }

  inline T kdtree_get_pt(const size_t idx, size_t dim) const
  {
      return states[idx].p[dim];
  }

  template <class BBox>
  inline T kdtree_get_bbox(BBox) const { return false; }
};

#endif // POINTCLOUD_HPP
