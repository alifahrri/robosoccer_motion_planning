#include <cmath>
#include <memory>
#include "kdtree.hpp"
#include "rrtstar.hpp"
#include "rrtvisual.hpp"
#include "integrator2d.hpp"
#include "environment.hpp"

#define SAMPLE_SCALE 5.0

namespace Geometric {

typedef Point<double,2> Point2D;
typedef PointCloud<double,2> PointCloud2D;
typedef KDTree<PointCloud2D,2,double,Point2D> KDTree2D;
typedef double CostType;

struct State2D
{
  // State3D() : p(Point3D()), c(CostType(0.0)) {}
  State2D() {}
  // State3D(Point3D *p, CostType *c) : p(p), c(c) {}
  State2D(std::shared_ptr<Point2D> p, std::shared_ptr<CostType> c) : p(p), c(c) {}
  bool operator==(State2D &rhs) {
    auto same = true;
    for(size_t i=0; i<2; i++) {
      same = ((*p)[i] == (*(rhs.p))[i]);
      if(!same) break;
    }
    return same;
  }
  std::shared_ptr<Point2D> p;
  std::shared_ptr<CostType> c;
  double operator()(size_t i) const {
    return (*p)[i];
  }
  CostType cost() const { return *c; }
  void setCost(const CostType &cost) { *c = cost; }
};

struct Connector2D
{
  typedef std::array<Point2D,2> Edge;
  Connector2D() {}
  Edge operator()(const State2D &s0, const State2D &s1) const {
    Edge e;
    e.front() = *s0.p;
    e.back() = *s1.p;
    return e;
  }
} connector;

struct Cost2D
{
  typedef CostType Scalar;
  Cost2D() {}
  Scalar operator()(const State2D &s0, const State2D &s1)
  {
    auto dx = s1.p->p[0]-s0.p->p[0];
    auto dy = s1.p->p[1]-s0.p->p[1];
    return std::sqrt(dx*dx+dy*dy);
  }
} cost2d;

struct Tree2D
{
  typedef int Index;
  typedef std::vector<Index> IndexList;
  typedef State2D State;

  Tree2D() {}
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

  Index insert(const State& s, const Index &idx, const Connector2D::Edge &e)
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

  void setEdge(const Index &node, const Connector2D::Edge &e) {

  }

  State operator()(const Index &i)
  {
    State s(std::make_shared<Point2D>(tree(i)), std::make_shared<CostType>(costs[i]));
    return s;
  }

  std::vector<CostType> costs;
  KDTree2D tree;
  IndexList parent;
} tree2d;

#define SAMPLE_X0 (11.0)
#define SAMPLE_X1 (7.0)
#define SAMPLE_X2 (1.5)
#define SAMPLE_X3 (1.5)

struct CollisionChecker
{
  CollisionChecker() {
    for(int i=0; i<2; i++)
      twister_engine[i] = std::mt19937_64(rd());
    dist[0] = new std::uniform_real_distribution<>(-SAMPLE_X0,SAMPLE_X0);
    dist[1] = new std::uniform_real_distribution<>(-SAMPLE_X1,SAMPLE_X1);
  }
  void setRandomObstacles() {
    for(auto& o : env.obs) {
      std::get<0>(o) = (*dist[0])(twister_engine[0]);
      std::get<1>(o) = (*dist[1])(twister_engine[1]);
    }
  }
  bool operator() (const Connector2D::Edge &e) {
    return env.collide<0,1>(e.front(), e.back());
  }
  bool operator() (const State2D &s0, const State2D &s1) {
    // for now, obstacle-free env
    return env.collide<0,1>(s0,s1);
  }
  constexpr static int obs_count = 9;
  Robosoccer<CostType,obs_count> env;
  std::random_device rd;
  std::mt19937_64 twister_engine[2];
  std::uniform_real_distribution<> *dist[2];
} checker;

struct Sampler
{
  Sampler() { twister_engine = std::mt19937_64(rd()); }
  State2D operator()()
  {
    std::shared_ptr<Point2D> pt(new Point2D());
    std::shared_ptr<CostType> c(new CostType(0.0));
    pt->p[0] = (dist[0](twister_engine)-0.5) * 2 * SAMPLE_X0;
    pt->p[1] = (dist[1](twister_engine)-0.5) * 2 * SAMPLE_X1;
    return State2D(pt,c);
  }
  std::random_device rd;
  std::mt19937_64 twister_engine;
  std::uniform_real_distribution<> dist[2];
} sampler;

struct NeighborRadius
{
#define SOME_CONSTANT (50.0)
  NeighborRadius() { s = std::pow(2,2)*(1+1/2)*SAMPLE_SCALE*SAMPLE_SCALE*SOME_CONSTANT; }
  double operator()(const Tree2D::Index &i) {
    return s*std::log(i+1)/(i+1);
  }
  double s;
} radius;

struct GoalChecker
{
  GoalChecker() {}
  bool operator()(const State2D &state) {
    // for now, iterate indefinetly
    return false;
  }
} goal;

typedef RRTStar<Tree2D,Cost2D,Sampler,NeighborRadius,CollisionChecker,GoalChecker,Connector2D> RRTStar2D;

RRTStar2D rrtstar_2d(tree2d, cost2d, sampler, checker, radius, goal, connector);
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"rrtstar_geometric_3d");
  ros::NodeHandle node;
  auto p0 = Geometric::Point2D();
  p0.p[0] = 0.0;
  p0.p[1] = 0.0;
  auto c = Geometric::CostType(0.0);
  auto &tree = Geometric::tree2d;
  auto &geom_rrt = Geometric::rrtstar_2d;
  auto start = Geometric::State2D(std::make_shared<Geometric::Point2D>(p0),std::make_shared<Geometric::CostType>(c));
  Geometric::checker.setRandomObstacles();
  RRTVisual vis(node);
  geom_rrt.setStart(start);
  ros::Rate rate(10.0f);
  ros::Duration duration(0,1);
  while(ros::ok()) {
    geom_rrt.grow();
    auto tree_size = tree.tree.size();
    ROS_INFO("tree size : %d;", tree_size);
    if((tree_size % 100) == 0) {
      vis.set_nodes(tree.tree.cloud.states,tree.parent,2,tree.tree.size());
      auto r = Geometric::checker.env.collision_radius;
      for(const auto &o : Geometric::checker.env.obs)
        vis.add_obstacles(std::get<0>(o),std::get<1>(o),r);
      vis.delete_all();
      vis.publish();
      vis.clear();
    }
    // rate.sleep();
    duration.sleep();
  }
  return 0;
}
