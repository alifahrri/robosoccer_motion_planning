#ifndef RANDOM_HPP
#define RANDOM_HPP

#include <cstdlib>
#include <random>

template<size_t dim, typename scalar = double>
struct RandomGen
{
  RandomGen& operator = (const RandomGen &rv) {
    for(size_t i=0; i<dim; i++) {
      if(dist[i]) *dist[i] = rv.dist[i];
      else dist[i] = new std::uniform_real_distribution<scalar>(rv.dist[i]->a(),rv.dist[i]->b());
      twister[i] = rv.twister[i];
    }
    return *this;
  }

  RandomGen(std::initializer_list<scalar> min, std::initializer_list<scalar> max) {
    auto i = 0;
    std::random_device rd;
    std::array<scalar,dim> a, b;
    for(auto v : min) {
      if(i==dim) break;
      a[i++] = v;
    }
    for(auto v : max) {
      if(i==dim) break;
      b[i++] = v;
    }
    for(size_t i=0; i<dim; i++) {
      twister[i] = std::mt19937_64(rd());
      dist[i] = new std::uniform_real_distribution<scalar>(a[i],b[i]);
    }
  }

  template <typename ArrayLike>
  inline
  void operator ()(ArrayLike &array) {
    for(size_t i=0; i<dim; i++)
      array[i] = (*dist[i])(twister[i]);
  }

  inline
  scalar operator()(size_t i) {
    if(i < dim) return (*dist[i])(twister[i]);
    else return scalar(0);
  }

  std::mt19937_64 twister[dim];
  std::uniform_real_distribution<scalar> *dist[dim];
};

#endif // RANDOM_HPP
