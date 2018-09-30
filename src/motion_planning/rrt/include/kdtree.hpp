#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <nanoflann.hpp>
#include "pointcloud.hpp"

using namespace nanoflann;

template <typename Container, int n, typename Scalar = typename Container::Scalar, typename State = typename Container::State>
class KDTree
{
  template<typename T> using vector = std::vector<T>;
  template<typename T, typename U> using pair = std::pair<T,U>;
protected:
    typedef KDTreeSingleIndexDynamicAdaptor<
            L2_Simple_Adaptor<Scalar,Container>,
            Container,n>
    KDTreeIndex;

public:
  KDTree() {}
  void clear()
  {
    cloud.clear();
    delete index;
    index = new KDTreeIndex(n, cloud, KDTreeSingleIndexAdaptorParams());
  }
  size_t size() { return cloud.size(); }

  const State& operator ()(const size_t &i) const { return cloud[i]; }
  State& operator ()(const size_t &i) {
    return cloud[i];
  }

  Point<Scalar,n> nearest(const State &p, size_t *nearest_index = nullptr, size_t n_neighbor = 1)
  {
    nanoflann::KNNResultSet<Scalar> result_set(1);
    size_t indices;
    Scalar distance;
    Scalar query_pts[n] = p.p;
    result_set.init(&indices, &distance);
    index->findNeighbors(result_set, query_pts, nanoflann::SearchParams(10));
    if(indices)
    {
      *nearest_index = indices;
    }
    return cloud[indices];
  }

  vector<pair<size_t,Scalar>> nearest(const State &p, Scalar radius)
  {
    vector<pair<size_t,Scalar>> ret_matches;
    nanoflann::SearchParams params;

    auto query_pt = p.data();

    nanoflann::RadiusResultSet<Scalar,size_t> result_set(radius, ret_matches);

    index->findNeighbors(result_set, query_pt, params);
    return ret_matches;
  }
  void addPoint(const State &point)
  {
    cloud.push_back(point);
    index->addPoints(cloud.size()-1,cloud.size()-1);
  }

public:
  // PointCloud<scalar,n> cloud;
  Container cloud;
  KDTreeIndex *index = new KDTreeIndex(n, cloud, KDTreeSingleIndexAdaptorParams());
};
#endif // KDTREE_HPP
