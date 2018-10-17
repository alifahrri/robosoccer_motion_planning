#include <cmath>
#include <memory>
#include "kdtree.hpp"
#include "rrtstar.hpp"
#include "rrtvisual.hpp"
#include "integrator2d.hpp"

#define SAMPLE_SCALE 5.0

namespace Geometric {

typedef Point<double,3> Point3D;
typedef PointCloud<double,3> PointCloud3D;
typedef KDTree<PointCloud3D,3,double,Point3D> KDTree3D;
typedef double CostType;

struct State3D
{
  // State3D() : p(Point3D()), c(CostType(0.0)) {}
  State3D() {}
  // State3D(Point3D *p, CostType *c) : p(p), c(c) {}
  State3D(std::shared_ptr<Point3D> p, std::shared_ptr<CostType> c) : p(p), c(c) {}
  bool operator==(State3D &rhs) {
    auto same = true;
    for(size_t i=0; i<3; i++) {
      same = ((*p)[i] == (*(rhs.p))[i]);
      if(!same) break;
    }
    return same;
  }
  std::shared_ptr<Point3D> p;
  std::shared_ptr<CostType> c;
  CostType cost() const { return *c; }
  void setCost(const CostType &cost) { *c = cost; }
};

struct Cost3D
{
  typedef CostType Scalar;
  Cost3D() {}
  Scalar operator()(const State3D &s0, const State3D &s1)
  {
    auto dx = s1.p->p[0]-s0.p->p[0];
    auto dy = s1.p->p[1]-s0.p->p[1];
    auto dz = s1.p->p[2]-s0.p->p[2];
    return std::sqrt(dx*dx+dy*dy+dz*dz);
  }
} cost3d;

struct Connector3D
{
  typedef std::array<Point3D,2> Edge;
  Connector3D() {}
  Edge operator()(const State3D &s0, const State3D &s1) const {
    Edge e;
    e.front() = *s0.p;
    e.back() = *s1.p;
    return e;
  }
} connector;

struct Tree3D
{
  typedef int Index;
  typedef std::vector<Index> IndexList;
  typedef State3D State;

  Tree3D() {}
  IndexList nearest(const State &s, const double &radius)
  {
    IndexList ret;
    auto vp = tree.nearest(*(s.p),radius);
    for(const auto& p : vp)
      ret.push_back(Index(p.first));
    return ret;
  }

  Index insert(const State& s, const Index &idx)
  {
    tree.addPoint(*(s.p));
    costs.push_back(*(s.c));
    parent.push_back(idx);
    return tree.size()-1;
  }

  Index insert(const State& s, const Index &idx, const Connector3D::Edge &e)
  {
    tree.addPoint(*(s.p));
    costs.push_back(*(s.c));
    parent.push_back(idx);
    return tree.size()-1;
  }

  void reset()
  {
    costs.clear();
    parent.clear();
    tree.clear();
  }

  void setParent(const Index& node, const Index &p)
  {
    parent[node] = p;
  }

  void setEdge(const Index &node, const Connector3D::Edge &e) {

  }

  State operator()(const Index &i)
  {
    State s(std::make_shared<Point3D>(tree(i)), std::make_shared<CostType>(costs[i]));
    return s;
  }

  std::vector<CostType> costs;
  KDTree3D tree;
  IndexList parent;
} tree3d;

struct CollisionChecker
{
  CollisionChecker() {}
  bool operator ()(const Connector3D::Edge &e) {
    // for now, obstacle-free env
    return false;
  }
  bool operator() (const State3D &s0, const State3D &s1) {
    // for now, obstacle-free env
    return false;
  }
} checker;

struct Sampler
{
  Sampler() { twister_engine = std::mt19937_64(rd()); }
  State3D operator()()
  {
    std::shared_ptr<Point3D> pt(new Point3D());
    std::shared_ptr<CostType> c(new CostType(0.0));
    pt->p[0] = (dist[0](twister_engine)-0.5) * SAMPLE_SCALE;
    pt->p[1] = (dist[1](twister_engine)-0.5) * SAMPLE_SCALE;
    pt->p[2] = dist[2](twister_engine) * SAMPLE_SCALE;
    return State3D(pt,c);
  }
  std::random_device rd;
  std::mt19937_64 twister_engine;
  std::uniform_real_distribution<> dist[3];
} sampler;

struct NeighborRadius
{
  NeighborRadius() { s = std::pow(2,3)*(1+1/3)*SAMPLE_SCALE*SAMPLE_SCALE*SAMPLE_SCALE; }
  double operator()(const Tree3D::Index &i) {
    return s*std::log(i+1)/(i+1);
  }
  double s;
} radius;

struct GoalChecker
{
  GoalChecker() {}
  bool operator()(const State3D &state) {
    // for now, iterate indefinetly
    return false;
  }
} goal;

typedef RRTStar<Tree3D,Cost3D,Sampler,NeighborRadius,CollisionChecker,GoalChecker,Connector3D> RRTStar3D;

RRTStar3D rrtstar_3d(tree3d, cost3d, sampler, checker, radius, goal, connector);
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"rrtstar_geometric_3d");
  ros::NodeHandle node;
  auto p0 = Geometric::Point3D();
  p0.p[0] = 0.0;
  p0.p[1] = 0.0;
  p0.p[2] = 0.0;
  auto c = Geometric::CostType(0.0);
  auto &tree = Geometric::tree3d;
  auto &geom_rrt = Geometric::rrtstar_3d;
  auto start = Geometric::State3D(std::make_shared<Geometric::Point3D>(p0),std::make_shared<Geometric::CostType>(c));
  RRTVisual vis(node);
  geom_rrt.setStart(start);
  ros::Rate rate(10.0f);
  while(ros::ok()) {
    geom_rrt.grow();
    vis.set_nodes(tree.tree.cloud.states,tree.parent,3,tree.tree.size());
    vis.delete_all();
    vis.publish();
    vis.clear();
    rate.sleep();
  }
  return 0;
}
