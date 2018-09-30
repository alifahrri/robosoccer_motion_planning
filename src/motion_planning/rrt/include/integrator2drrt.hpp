#ifndef INTEGRATOR2DRRT_HPP
#define INTEGRATOR2DRRT_HPP

#include <cmath>
#include <memory>
#include "environment.hpp"
#include "logger.hpp"
#include "states.hpp"
#include "kdtree.hpp"
#include "rrtstar.hpp"
#include "rrtvisual.hpp"
#include "integrator2d.hpp"
#include "random.hpp"

// #define NO_OBS

namespace Kinodynamic {

constexpr int segment = 10;

typedef Models::Type scalar;
typedef State<scalar,Models::n> state_t;
typedef States<scalar,Models::n> states_t;
typedef KDTree<states_t,4,Models::Type,state_t> KDTreeInt2D;
typedef double CostType;
typedef Models::Integrator2DCost CostFN;

template<typename Scalar = double, typename State = state_t, int n = segment>
struct Trajectory
{
  Trajectory(const Models::Integrator2DTrajectorySolver::Trajectory &trajectory)
  {
    for(size_t i=0; i<=n; i++)
      trj[i] = std::make_tuple(std::get<0>(trajectory[i]),std::get<1>(trajectory[i]));
  }
  Trajectory(const Trajectory& t) {
    for(size_t i=0; i<=n; i++)
      trj[i] = t.trj[i];
  }
  Trajectory(){
    for(size_t i=0; i<=n; i++)
      trj[i] = std::make_tuple(Scalar(0.0),state_t());
  }
  Trajectory(const state_t &s){
    for(size_t i=0; i<=n; i++)
      trj[i] = std::make_tuple(Scalar(0.0),s);
  }
  std::array<Scalar,n+1> time() const {
    std::array<Scalar,n+1> ret;
    for(size_t i=0; i<=n; i++)
      ret[i] = std::get<0>(trj[i]);
    return ret;
  }
  std::array<state_t,n+1> path() const {
    std::array<state_t,n+1> ret;
    for(size_t i=0; i<=n; i++)
      ret[i] = std::get<1>(trj[i]);
    return ret;
  }
  const std::tuple<Scalar,state_t>& operator[](size_t i) const { return trj[i]; }
  std::tuple<Scalar,state_t>& operator[](size_t i) { return trj[i]; }
  std::array<std::tuple<Scalar,state_t>,n+1> trj;
};
template <typename T, typename S, int n> using Trajectories = std::vector<Trajectory<T,S,n>>;

struct CostInt2D
{
  typedef scalar Scalar;
  CostInt2D(Models::Integrator2DTrajectorySolver &solver) : solver(solver) {}
  Scalar operator()(const state_t& s0, const state_t& s1) const
  {
    Models::Integrator2DSS::StateType xi;
    Models::Integrator2DSS::StateType xf;
    for(size_t i=0; i<4; i++) {
      xi(i) = s0(i);
      xf(i) = s1(i);
    }
    return std::get<1>(solver.cost(xi,xf));
  }
  Models::Integrator2DTrajectorySolver &solver;
} cost_int2d(Models::integrator2d_trj_solver);

struct Connector
{
  typedef Trajectory<scalar,state_t,segment> Edge;

  Connector(Models::Integrator2DTrajectorySolver &solver) : solver(solver) {}

  Edge operator()(const state_t &s0, const state_t &s1) const {
    Models::Integrator2DSS::StateType xi;
    Models::Integrator2DSS::StateType xf;
    for(size_t i=0; i<4; i++) {
      xi(i) = s0(i);
      xf(i) = s1(i);
    }
    auto trajectory = solver.solve<segment>(xi, xf);
    return Trajectory<scalar,state_t,segment>(trajectory);
  }

  Models::Integrator2DTrajectorySolver &solver;
} connector(Models::integrator2d_trj_solver);

struct TreeInt2D
{
  typedef int Index;
  typedef std::vector<Index> IndexList;
  typedef state_t State;

  TreeInt2D() {}

  IndexList nearest(const State &s0, const scalar& radius)
  {
    IndexList ret;
    auto vp = tree.nearest(s0,radius);
    for(const auto& p : vp)
      ret.push_back(Index(p.first));
    return ret;
  }

  Index insert(const State &s, const Index &idx)
  {
    Index id = tree.size();
    auto e = Trajectory<scalar,state_t,segment>(s);
    tree.addPoint(s);
    setParent(id, idx);
    setEdge(id, e);
    return id;
  }

  Index insert(const State &s, const Index &idx, const Connector::Edge &e)
  {
    Index id = tree.size();
    tree.addPoint(s);
    parent.push_back(idx);
    trajectories.push_back(e);
    return id;
  }

  void reset()
  {
    tree.clear();
    parent.clear();
    trajectories.clear();
  }

  void setParent(const Index &node, const Index &p)
  {
    if(parent.size() < node+1)
      parent.push_back(p);
    else parent.at(node) = p;
  }

  void setEdge(const Index &n, const Connector::Edge &e)
  {
    if(trajectories.size() < n+1)
      trajectories.push_back(e);
    else trajectories.at(n) = e;
  }

  const State& operator()(const Index &i) const
  {
    return tree(i);
  }

  State& operator()(const Index &i)
  {
    return tree(i);
  }

  void dump_text(const std::string &node_file, const std::string &parent_file, const std::string &trajectory_file)
  {
    Logger logger;
    for(const auto& n : tree.cloud.states)
      logger << "("
             << n(0) << "," << n(1) << ","
             << n(2) << "," << n(3) << ","
             << n.cost()
             << ")\n";
    logger.save(node_file);
    logger.clear();

    for(const auto &p : parent)
      logger << p << "\n";
    logger.save(parent_file);
    logger.clear();

    for(const auto &trj : trajectories) {
      for(size_t i=0; i<=segment; i++)
      {
        const auto &t = trj[i];
        auto p = std::get<1>(t);
        logger << "("
               << std::get<0>(t) << ","
               << p(0) << "," << p(1) << ","
               << p(2) << "," << p(3)
               << ") ";
      }
      logger << "\n";
    }
    logger.save(trajectory_file);
    logger.clear();
  }

  void from_text(const std::string &node_file, const std::string &parent_file, const std::string &trajectory_file)
  {
    std::ifstream tree_stream(node_file);
    std::ifstream parent_stream(parent_file);
    std::ifstream trajectory_stream(trajectory_file);
    std::string line;

    tree.clear();
    parent.clear();
    trajectories.clear();

    while(std::getline(tree_stream, line)) {
      line.erase(std::remove(line.begin(), line.end(), '('), line.end());
      line.erase(std::remove(line.begin(), line.end(), ')'), line.end());
      std::string s;
      std::vector<std::string> values;
      std::istringstream ss(line);
      while(std::getline(ss,s,','))
        values.push_back(s);
      state_t state;
      // std::cout << line << std::endl;
      for(size_t i=0; i<4; i++)
        state[i] = std::stod(values[i]);
      state.setCost(std::stod(values.back()));
      tree.addPoint(state);
    }
    while(std::getline(parent_stream, line))
      parent.push_back(std::stoi((line)));
    while(std::getline(trajectory_stream, line)) {
      std::string s;
      std::vector<std::string> values;
      std::istringstream ss(line);
      Trajectory<scalar,state_t,segment> trajectory;
      while(std::getline(ss, s, ' '))
        values.push_back(s);
      for(size_t i=0; i<values.size(); i++) {
        auto& str = values.at(i);
        str.erase(std::remove(str.begin(), str.end(), '('), str.end());
        str.erase(std::remove(str.begin(), str.end(), ')'), str.end());
        std::string sstr;
        std::istringstream sss(str);
        std::vector<std::string> states;
        state_t state;
        while(std::getline(sss, sstr, ','))
          states.push_back(sstr);
        // std::cout << str << std::endl;
        auto time = std::stod(states.front());
        for(size_t k=1; k<=4; k++)
          state[k-1] = std::stold(states.at(k));
        trajectory.trj.at(i) = std::make_tuple(time,state);
      }
      trajectories.push_back(trajectory);
    }

    tree_stream.close();
    parent_stream.close();
    trajectory_stream.close();
  }

  Trajectories<scalar,state_t,segment> get_trajectory(Index idx) {
    auto i = idx;
    Trajectories<scalar,state_t,segment> sol;
    while(i>0) {
      sol.push_back(trajectories[i]);
      i = parent[i];
    }
    return sol;
  }

  IndexList parent;
  KDTreeInt2D tree;
  Trajectories<scalar,state_t,segment> trajectories;
} tree_int2d;

#define SAMPLE_X0 (11.0)
#define SAMPLE_X1 (7.0)
#define SAMPLE_X2 (1.5)
#define SAMPLE_X3 (1.5)

constexpr int obs_count = 9;
Robosoccer<scalar,obs_count> robosoccer_env;
DynamicRobosoccer<scalar,obs_count> dynamic_soccer_env;

struct CollisionTimeSpaceChecker
{
  CollisionTimeSpaceChecker(Models::Integrator2DTrajectorySolver& solver, DynamicRobosoccer<scalar,obs_count> &env)
    : solver(solver), env(env)
  {
    rg = new RandomGen<4,scalar>(
    {-SAMPLE_X0,-SAMPLE_X1,-SAMPLE_X2,-SAMPLE_X3},
    {SAMPLE_X0,SAMPLE_X1,SAMPLE_X2,SAMPLE_X3}
          );
  }

  void setRandomObstacles() {
    for(auto& o : env.obs) {
      (*rg)(o);
      // std::get<0>(o) = (*rg)(0);
      // std::get<1>(o) = (*rg)(1);
      // std::get<2>(o) = (*rg)(2);
      // std::get<3>(o) = (*rg)(3);
    }
  }

  bool operator() (const Connector::Edge &e) {
    auto collision = false;
    for(size_t i=1; i<e.trj.size(); i++) {
      const auto &ts1 = e.trj[i];
      const auto &ts0 = e.trj[i-1];
      const auto &s1 = std::get<1>(ts1);
      const auto &t1 = std::get<0>(ts1);
      const auto &s0 = std::get<1>(ts0);
      const auto &t0 = std::get<0>(ts0);
      collision = env.collide<0,1,2,3>(s0,s1,t0,t1);
      if(collision) break;
    }
    return collision;
  }

  bool operator() (const state_t &s0, const state_t &s1) const
  {

  }

  Models::Integrator2DTrajectorySolver &solver;
  DynamicRobosoccer<scalar,obs_count> &env;
  RandomGen<4,scalar> *rg;
} checker_time_space(Models::integrator2d_trj_solver, dynamic_soccer_env);

struct CollisionChecker
{
  CollisionChecker(Models::Integrator2DTrajectorySolver &solver, Robosoccer<scalar,obs_count> &env)
    : solver(solver), env(env)
  {
    rg = new RandomGen<2,scalar>({-SAMPLE_X0,-SAMPLE_X1},{SAMPLE_X0,SAMPLE_X1});
    // for(int i=0; i<2; i++)
    // twister_engine[i] = std::mt19937_64(rd());
    // dist[0] = new std::uniform_real_distribution<>(-SAMPLE_X0,SAMPLE_X0);
    // dist[1] = new std::uniform_real_distribution<>(-SAMPLE_X1,SAMPLE_X1);
  }

  void dump_text(const std::string &env_file, const std::string &collision_file) {
    Logger logger;
    for(const auto &o : env.obs)
      logger << std::get<0>(o) << " " << std::get<1>(o) << '\n';
    logger.save(env_file);

    /*
    logger.clear();
    for(const auto& c : collisions)
      logger << "(" << std::get<0>(c) << "," << std::get<1>(c) << ")\n";
    logger.save(collision_file);
    */
  }

  void from_text(const std::string &file) {
    std::string str;
    std::ifstream stream(file);
    int i=0;
    while(std::getline(stream, str)) {
      std::vector<std::string> vstr;
      std::istringstream istr(str);
      std::string sstr;
      while(std::getline(istr, sstr, ' '))
        vstr.push_back(sstr);
      env.obs.at(i) = std::make_tuple(std::stold(vstr.at(0)),std::stold(vstr.at(1)));
      i++;
    }
    stream.close();
  }

  void setRandomObstacles() {
    for(auto& o : env.obs) {
      std::get<0>(o) = (*rg)(0);
      std::get<1>(o) = (*rg)(1);
      // std::get<0>(o) = (*dist[0])(twister_engine[0]);
      // std::get<1>(o) = (*dist[1])(twister_engine[1]);
    }
    collisions.clear();
  }

  bool operator() (const Connector::Edge &e) {
#ifdef NO_OBS
    return false;
#else
    auto path = e.path();
    bool collision = false;
    for(size_t i=1; i<path.size(); i++) {
      const auto &p0 = path[i-1];
      const auto &p1 = path[i];
      if(env.collide<state_t,0,1>(p0) || env.collide<state_t,0,1>(p1))
        collision = true;
      if(env.collide<0,1>(path[i-1],path[i]))
        collision = true;
      if(collision)
        break;
    }
    /*
    if(collision)
      collisions.push_back(std::make_tuple(path.front(),path.back()));
      */
    return collision;
#endif
  }

  bool operator() (const state_t &s0, const state_t &s1) const
  {
#ifdef NO_OBS
    return false;
#else
    bool collision = false;

    if(env.collide<state_t,0,1>(s1))
      collision = true;

    if(env.collide<state_t,0,1>(s0))
      collision = true;

    if(!collision) {
      auto trajectory = solver.solve<segment>(s0,s1);
      for(size_t i=1; i<trajectory.size(); i++) {
        const auto &t0 = std::get<1>(trajectory[i-1]);
        const auto &t1 = std::get<1>(trajectory[i]);
        if(env.collide<0,1>(t0,t1)) {
          collision = true;
          break;
        }
      }
    }
    return collision;
#endif
  }

  std::vector<std::tuple<state_t,state_t>> collisions;
  Models::Integrator2DTrajectorySolver &solver;
  Robosoccer<scalar,obs_count> &env;
  RandomGen<2,scalar> *rg;
  // std::random_device rd;
  // std::mt19937_64 twister_engine[2];
  // std::uniform_real_distribution<> *dist[2];
} checker(Models::integrator2d_trj_solver, robosoccer_env);

struct Sampler
{
  Sampler(Robosoccer<scalar,obs_count> &env) : env(env) {
    rg = new RandomGen<4,scalar>(
    {-SAMPLE_X0,-SAMPLE_X1,-SAMPLE_X2,-SAMPLE_X3},
    {SAMPLE_X0,SAMPLE_X1,SAMPLE_X2,SAMPLE_X3}
          );
    // for(int i=0; i<4; i++)
    // twister_engine[i] = std::mt19937_64(rd());
    // dist[0] = new std::uniform_real_distribution<>(-SAMPLE_X0,SAMPLE_X0);
    // dist[1] = new std::uniform_real_distribution<>(-SAMPLE_X1,SAMPLE_X1);
    // dist[2] = new std::uniform_real_distribution<>(-SAMPLE_X2,SAMPLE_X2);
    // dist[3] = new std::uniform_real_distribution<>(-SAMPLE_X3,SAMPLE_X3);
  }
  state_t operator()()
  {
    // for(int i=0; i<4; i++)
      // s(i) = (*rg)(i);
    (*rg)(s);
    while(env.collide<state_t,0,1>(s))
      s = (*this)();
    return s;
  }

  state_t s;
  Robosoccer<scalar,obs_count> &env;
  RandomGen<4,scalar> *rg;
  // std::random_device rd;
  // std::mt19937_64 twister_engine[4];
  // std::uniform_real_distribution<> *dist[4];
} sampler(robosoccer_env);

struct GoalChecker
{
  GoalChecker(Robosoccer<scalar,obs_count> &env) : env(env) {
    rg = new RandomGen<4,scalar>({-SAMPLE_X0,-SAMPLE_X1,0.0,0.0},{SAMPLE_X0,SAMPLE_X1,0.0,0.0});
    // for(int i=0; i<2; i++)
    // twister_engine[i] = std::mt19937_64(rd());
    // dist[0] = new std::uniform_real_distribution<>(-SAMPLE_X0,SAMPLE_X0);
    // dist[1] = new std::uniform_real_distribution<>(-SAMPLE_X1,SAMPLE_X1);
  }
  bool operator()(const state_t &state) {
    // for now, iterate indefinetly
    return false;
  }
  state_t randomGoal() {
    state_t s;
    // for(int i=0; i<2; i++)
    // s(i) = (*dist[i])(twister_engine[i]);
    s(0) = (*rg)(0); s(1) = (*rg)(1);
    s(2) = s(3) = 0.0;
    while(env.collide<state_t,0,1>(s))
      s = this->randomGoal();
    return s;
  }
  Robosoccer<scalar,obs_count> &env;
  RandomGen<4,scalar> *rg;
  // std::random_device rd;
  // std::mt19937_64 twister_engine[2];
  // std::uniform_real_distribution<> *dist[2];
} goal(robosoccer_env);

struct NeighborRadius
{
#define SOME_CONSTANT (10.0)
#define SAMPLE_VOLUME (SAMPLE_X0*SAMPLE_X1*SAMPLE_X2*SAMPLE_X3)*(2*2*2*2)
  NeighborRadius() { s = std::pow(2,4)*(1+1/4)*SAMPLE_VOLUME; }
  double operator()(const TreeInt2D::Index &i) {
    return s*scale*std::log(i+1)/(i+1);
  }
  double s;
  double scale = SOME_CONSTANT;
} radius;

typedef RRTStar
<TreeInt2D,CostInt2D,Sampler,NeighborRadius,CollisionChecker,GoalChecker,Connector>
RRTStarInt2D;

typedef RRTStar
<TreeInt2D,CostInt2D,Sampler,NeighborRadius,CollisionTimeSpaceChecker,GoalChecker,Connector>
RRTStarInt2DTimeSpaceObs;

RRTStarInt2D rrtstar_int2d(tree_int2d, cost_int2d, sampler, checker, radius, goal, connector);
RRTStarInt2DTimeSpaceObs rrtstar_int2d_timespace_obs(tree_int2d, cost_int2d, sampler, checker_time_space, radius, goal, connector);
}

#endif // INTEGRATOR2DRRT_HPP
