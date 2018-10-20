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
constexpr int obs_count = 9;
// collision check segment for dynamic obs
constexpr int collision_segment = 10;

typedef Models::scalar scalar;
typedef State<scalar,Models::n> state_t;
typedef States<scalar,Models::n> states_t;
typedef KDTree<states_t,4,Models::scalar,state_t> KDTreeInt2D;
typedef double CostType;
typedef Models::Integrator2DCost CostFN;

template<typename Scalar = double, typename State = state_t, int dim = State::dim>
struct TimeState : std::tuple<Scalar,State>
{
  TimeState() {}
  TimeState<Scalar,State>& operator = (const std::tuple<Scalar,State>& rhs)
  {
    std::get<0>(*this) = std::get<0>(rhs);
    std::get<1>(*this) = std::get<1>(rhs);
    return *this;
  }

  template<typename Index, int time_idx = dim>
  Scalar operator()(Index idx) const {
    return (idx < dim ? std::get<1>(*this)(idx) : std::get<0>(*this));
  }
};

template<typename Scalar = double, typename State = state_t, int n = segment>
struct Trajectory : std::array<TimeState<Scalar,State>,n+1>
{
  Trajectory(const Models::Integrator2DTrajectorySolver::Trajectory &trajectory)
  {
    for(size_t i=0; i<=n; i++)
      (*this)[i] = std::make_tuple(Scalar(std::get<0>(trajectory[i])),State(std::get<1>(trajectory[i])));
  }
  Trajectory(const Trajectory& t) {
    for(size_t i=0; i<=n; i++)
      (*this)[i] = t[i];
  }
  Trajectory(){
    for(size_t i=0; i<=n; i++)
      (*this)[i] = std::make_tuple(Scalar(0.0),state_t());
  }
  Trajectory(const state_t &s){
    for(size_t i=0; i<=n; i++)
      (*this)[i] = std::make_tuple(Scalar(0.0),s);
  }
  std::array<Scalar,n+1> time() const {
    std::array<Scalar,n+1> ret;
    for(size_t i=0; i<=n; i++)
      ret[i] = std::get<0>((*this)[i]);
    return ret;
  }
  Trajectory<Scalar,State,n> operator + (const Scalar &t) {
    auto trajectory = *this;
    for(auto & trj: trajectory)
      std::get<0>(trj) += t;
    return trajectory;
  }
  std::array<state_t,n+1> path() const {
    std::array<state_t,n+1> ret;
    for(size_t i=0; i<=n; i++)
      ret[i] = std::get<1>((*this)[i]);
    return ret;
  }
  // const std::tuple<Scalar,state_t>& operator[](size_t i) const { return trj[i]; }
  // std::tuple<Scalar,state_t>& operator[](size_t i) { return trj[i]; }
  // std::array<std::tuple<Scalar,state_t>,n+1> trj;
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
};

struct TreeInt2D;

struct Connector
{
  typedef Trajectory<scalar,state_t,segment> Edge;

  Connector(Models::Integrator2DTrajectorySolver &solver, TreeInt2D &tree)
    : solver(solver), tree(tree) {}
  inline
  Edge operator()(const state_t &s0, const state_t &s1) ;
  inline
  Edge last_connection()
  {
    return e;
  }
  Edge e; // last connection
  Models::Integrator2DTrajectorySolver &solver;
  TreeInt2D &tree;
};

struct TreeInt2D
{
  typedef int Index;
  typedef std::vector<Index> IndexList;
  typedef state_t State;
  // typedef std::vector<std::reference_wrapper<State>> StateList;
  typedef std::vector<State> StateList;

  TreeInt2D() {}

  IndexList nearest(const State &s0, const scalar& radius)
  {
    IndexList ret;
    auto vp = tree.nearest(s0,radius);
    for(const auto& p : vp)
      ret.push_back(Index(p.first));
    return ret;
  }

  StateList states(const IndexList &indexes)
  {
    StateList ret;
    for(const auto &i : indexes)
      ret.push_back(tree(i));
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
    auto edge = e;
    // if(idx >= 0) {
      // auto t = std::get<0>(trajectories.at(idx).back());
      // edge = edge + t;
    // }
    trajectories.push_back(edge);
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
    if(int(parent.size()) < node+1)
      parent.push_back(p);
    else parent.at(node) = p;
  }

  void setEdge(const Index &n, const Connector::Edge &e)
  {
    auto edge = e;
    // if(parent.at(n) >= 0) {
      // auto t = std::get<0>(trajectories.at(parent.at(n)).back());
      // edge = edge + t;
    // }
    if(int(trajectories.size()) < n+1)
      trajectories.push_back(edge);
    else trajectories.at(n) = edge;
  }

  // const State& operator()(const Index &i) const
  // {
    // last_checked_idx = i;
    // return tree(i);
  // }

  State& operator()(const Index &i)
  {
    last_checked_idx = i;
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
        trajectory.at(i) = std::make_tuple(time,state);
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

  int last_checked_idx = -1;
  IndexList parent;
  KDTreeInt2D tree;
  Trajectories<scalar,state_t,segment> trajectories;
};

inline
Connector::Edge Connector::operator()(const state_t &s0, const state_t &s1) {
  // Models::Integrator2DSS::StateType xi;
  // Models::Integrator2DSS::StateType xf;
  // for(size_t i=0; i<4; i++) {
    // xi(i) = s0(i);
    // xf(i) = s1(i);
  // }
  auto trajectory = solver.solve<segment>(s0, s1);
  auto ti_idx = tree.last_checked_idx;
  auto t0 = 0.0;
  e = Trajectory<scalar,state_t,segment>(trajectory);
  if(ti_idx > 0) {
    t0 = std::get<0>(tree.trajectories.at(ti_idx).back());
    e = e + t0;
  }
  return e;
}

#define SAMPLE_X0 (11.0)
#define SAMPLE_X1 (7.0)
#define SAMPLE_X2 (1.5)
#define SAMPLE_X3 (1.5)

// for now, dont compile this on cuda
// @TODO : make this work on both
#ifndef __NVCC__

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
    }
  }

  bool operator() (const Connector::Edge &e) {
    auto collision = false;
    for(size_t i=1; i<e.size(); i++) {
      const auto &ts1 = e[i];
      const auto &ts0 = e[i-1];
      const auto &s1 = std::get<1>(ts1);
      const auto &t1 = std::get<0>(ts1);
      const auto &s0 = std::get<1>(ts0);
      const auto &t0 = std::get<0>(ts0);
      collision = env.collide<0,1>(s0,s1,t0,t1);
      if(collision) break;
    }
    return collision;
  }

//  bool operator() (const state_t &s0, const state_t &s1) const
//  {

//  }

  Models::Integrator2DTrajectorySolver &solver;
  DynamicRobosoccer<scalar,obs_count> &env;
  RandomGen<4,scalar> *rg;
};

struct CollisionChecker
{
  CollisionChecker(Models::Integrator2DTrajectorySolver &solver, Robosoccer<scalar,obs_count> &env)
    : solver(solver), env(env)
  {
    rg = new RandomGen<2,scalar>({-SAMPLE_X0,-SAMPLE_X1},{SAMPLE_X0,SAMPLE_X1});
  }

#ifndef __NVCC__
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
    }
    collisions.clear();
  }
#endif

  bool operator() (const Connector::Edge &e) {
#ifdef NO_OBS
    return false;
#else
    auto path = e.path();
    bool collision = false;
    for(size_t i=1; i<path.size(); i++) {
      const auto &p0 = path[i-1];
      const auto &p1 = path[i];
      if(env.collide<0,1>(p0) || env.collide<0,1>(p1))
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

    if(env.collide(s1))
      collision = true;

    if(env.collide(s0))
      collision = true;

    if(!collision) {
      auto trajectory = solver.solve<segment>(s0,s1);
      for(size_t i=1; i<trajectory.size(); i++) {
        const auto &t0 = std::get<1>(trajectory[i-1]);
        const auto &t1 = std::get<1>(trajectory[i]);
        if(env.collide(t0,t1)) {
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
};
#endif

template <typename Environment>
struct Sampler
{
  Sampler(Environment &env) : env(env) {
    rg = new RandomGen<4,scalar>(
    {-SAMPLE_X0,-SAMPLE_X1,-SAMPLE_X2,-SAMPLE_X3},
    {SAMPLE_X0,SAMPLE_X1,SAMPLE_X2,SAMPLE_X3});
  }

  inline
  state_t operator()()
  {
    (*rg)(s);
    // for now dont compile this on cuda
    // todo resolve
#ifndef __NVCC__
    while(env.collide(s))
      (*rg)(s);
#endif
    return s;
  }

  inline
  state_t last_sample()
  {
    return s;
  }

  state_t s;
  Environment &env;
  RandomGen<4,scalar> *rg = nullptr;
};

template <typename Environment>
struct GoalChecker
{
  GoalChecker(Environment &env) : env(env) {
    rg = new RandomGen<2,scalar>({-SAMPLE_X0,-SAMPLE_X1},{SAMPLE_X0,SAMPLE_X1});
  }
  inline
  bool operator()(const state_t &state) {
    // for now, iterate indefinetly
    return false;
  }
  state_t randomGoal() {
    state_t s;
    s(2) = s(3) = 0.0;
    (*rg)(s);
    // for now dont compile this on cuda
    // todo resolve
#ifndef __NVCC__
    while(env.collide(s))
      (*rg)(s);
#endif
    return s;
  }

  Environment &env;
  RandomGen<2,scalar> *rg;
};

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
};

TreeInt2D tree_int2d;

typedef Robosoccer<scalar,obs_count> StaticEnvironment;
typedef DynamicRobosoccer<scalar,obs_count> DynamicEnvironment;
StaticEnvironment robosoccer_env;
DynamicEnvironment dynamic_soccer_env;

// don't instantiate these object if use cuda
#ifndef __NVCC__
CostInt2D cost_int2d(Models::integrator2d_trj_solver);
Connector connector(Models::integrator2d_trj_solver, tree_int2d);
#endif

// automatic template class deduction, need c++17 (gcc >= 7)
#if (__GNUC__ < 7) || defined(__NVCC__)
GoalChecker<StaticEnvironment> goal(robosoccer_env);
GoalChecker<DynamicEnvironment> goal_dynamic_env(dynamic_soccer_env);
#else
GoalChecker goal(robosoccer_env);
GoalChecker goal_dynamic_env(dynamic_soccer_env);
#endif

// automatic template class deduction, need c++17 (gcc >= 7)
#if (__GNUC__ < 7) || defined(__NVCC__)
Sampler<StaticEnvironment> sampler(robosoccer_env);
Sampler<DynamicEnvironment> sampler_dynamic_env(dynamic_soccer_env);
#else
Sampler sampler(robosoccer_env);
Sampler sampler_dynamic_env(dynamic_soccer_env);
#endif

NeighborRadius radius;

// dont instantiate these object if compile using cuda
#ifndef __NVCC__
CollisionChecker checker(Models::integrator2d_trj_solver, robosoccer_env);
CollisionTimeSpaceChecker checker_time_space(Models::integrator2d_trj_solver, dynamic_soccer_env);

// automatic template class deduction, need c++17 (gcc >= 7)
#if defined(__GNUC__) && (__GNUC__ >= 7)
RRTStar rrtstar_int2d(tree_int2d, cost_int2d, sampler, checker, radius, goal, connector);
RRTStar rrtstar_int2d_timespace_obs(tree_int2d, cost_int2d, sampler_dynamic_env, checker_time_space, radius, goal_dynamic_env, connector);
#else
typedef RRTStar
<TreeInt2D,CostInt2D,Sampler<StaticEnvironment>,NeighborRadius,CollisionChecker,GoalChecker<StaticEnvironment>,Connector>
RRTStarInt2D;

typedef RRTStar
<TreeInt2D,CostInt2D,Sampler<DynamicEnvironment>,NeighborRadius,CollisionTimeSpaceChecker,GoalChecker<DynamicEnvironment>,Connector>
RRTStarInt2DTimeSpaceObs;

RRTStarInt2D rrtstar_int2d(tree_int2d, cost_int2d, sampler, checker, radius, goal, connector);
RRTStarInt2DTimeSpaceObs rrtstar_int2d_timespace_obs(tree_int2d, cost_int2d, sampler_dynamic_env, checker_time_space, radius, goal_dynamic_env, connector);
#endif

#endif
}


#endif // INTEGRATOR2DRRT_HPP
