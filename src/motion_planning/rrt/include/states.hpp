#ifndef STATES_HPP
#define STATES_HPP

#include <vector>
#include <eigen3/Eigen/Core>

template
<typename scalar, int n>
struct State : public Eigen::Matrix<scalar,n,1>
{
  State() {}
  scalar cost() const { return c; }
  void setCost(const scalar &c) { this->c = c; }
  scalar c = scalar(0.0);
  State& operator = (const Eigen::Matrix<scalar,n,1> &m)
  {
    for(size_t i=0; i<n; i++)
      (*this)(i) = m(i);
    return *this;
  }
  scalar& operator[](const int &i) {
    return (*this)(i);
  }
  const scalar& operator[](const int &i) const{
    return (*this)(i);
  }
};

template
<typename scalar, int n>
struct States
{
  States() {}

  inline State<scalar,n>& operator[](const size_t &i) { return states[i]; }
  inline const State<scalar,n>& operator[](const size_t &i) const { return states[i]; }
  inline void push_back(const State<scalar,n> &s) { states.push_back(s); }
  inline void clear() { states.clear(); }
  inline size_t size() const { return states.size(); }
  inline size_t kdtree_get_point_count() const { return states.size(); }
  inline scalar kdtree_get_pt(const size_t idx, size_t dim) const
  {
    return states[idx](dim);
  }
  template <class BBox>
  inline scalar kdtree_get_bbox(BBox) const { return false; }

  std::vector<State<scalar,n>> states;
};
#endif // STATES_HPP
