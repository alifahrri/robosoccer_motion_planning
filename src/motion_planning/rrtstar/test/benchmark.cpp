#include <benchmark/benchmark.h>
#include <vector>
#include "integrator2drrt.hpp"
#include "random.hpp"
#include "states.hpp"
#include "environment.hpp"

typedef Kinodynamic::GoalChecker<Kinodynamic::StaticEnvironment> GoalChecker;
typedef State<double,Models::n> state_t;
typedef RandomGen<2,double> RandomGen2;
typedef RandomGen<4,double> RandomGen4;
typedef Kinodynamic::Sampler<Kinodynamic::StaticEnvironment> Sampler;
typedef Robosoccer<> Environment;
typedef Models::Integrator2DOptTimeDiff TimeDiff;
typedef Models::Integrator2DOptTimeSolver TimeSolver;

#define TEST_NEIGHBOR (150)
#define TEST_NODES (2000)

static void opt_time_solver(benchmark::State &state) {
  // auto &time_diff = Models::integrator2d_opt_time_diff;
  auto &time_solver = Models::integrator2d_opt_time_solver;
  state_t s0, s1;
  s0(0) = 1.54; s0(1) = 0.153; s0(2) = 0.72; s0(3) = 0.15;
  s1(0) = 0.0; s1(1) = -1.2; s1(2) = -0.1; s1(3) = 0.0;

  for(auto _ : state)
    time_solver.solve(s0, s1);
}
BENCHMARK(opt_time_solver);

static void control(benchmark::State &state) {
  auto &connector = Kinodynamic::connector;
  auto &sampler = Kinodynamic::sampler_dynamic_env;
  auto s0 = sampler();
  auto s1 = sampler();

  for(auto _ : state)
    connector(s0, s1);
}
BENCHMARK(control);

static void collision_static_env(benchmark::State &state) {
  auto &env = Kinodynamic::robosoccer_env;
  auto &connector = Kinodynamic::connector;
  auto &collision = Kinodynamic::checker;
  auto &sampler = Kinodynamic::sampler;
  auto s0 = sampler();
  auto s1 = sampler();
  auto edge = connector(s0, s1);
  env.setRandomObstacles();

  for(auto _ : state)
    collision(edge);
}
BENCHMARK(collision_static_env);

static void collision_dynamic_env(benchmark::State &state) {
  auto &env = Kinodynamic::dynamic_soccer_env;
  auto &connector = Kinodynamic::connector;
  auto &collision = Kinodynamic::checker_time_space;
  auto &sampler = Kinodynamic::sampler_dynamic_env;
  auto s0 = sampler();
  auto s1 = sampler();
  auto edge = connector(s0, s1);
  env.setRandomObstacles();

  for(auto _ : state)
    collision(edge);
}
BENCHMARK(collision_dynamic_env);

static void tree_nearest_args(benchmark::internal::Benchmark *b) {
  for(size_t i=1; i<5000; i+=100)
    b->Arg(i);
}

static void tree_insert_args(benchmark::internal::Benchmark *b) {
  for(size_t i=1; i<3000; i+=100)
    b->Arg(i);
}

static void tree_nearest(benchmark::State &state) {
  auto &tree = Kinodynamic::tree_int2d;
  auto &radius = Kinodynamic::radius;
  auto xs = Kinodynamic::sampler();
  tree.reset();
  auto n = state.range(0);
  tree.insert(xs, -1);
  for(size_t i=0; i<n; i++) {
    auto x = Kinodynamic::sampler();
    tree.insert(x,0);
  }

  auto x = Kinodynamic::sampler();
  for(auto _ : state) {
    tree.nearest(x,radius(n));
  }
}
BENCHMARK(tree_nearest)->Apply(tree_nearest_args);

static void tree_insert(benchmark::State &state) {
  auto &tree = Kinodynamic::tree_int2d;
  tree.reset();
  auto n = state.range(0);

  for(auto _ : state) {
    for(size_t i=0; i<n; i++) {
      auto x = Kinodynamic::sampler();
      tree.insert(x,0);
    }
  }
}
BENCHMARK(tree_insert)->Apply(tree_insert_args);

static void cost(benchmark::State &state) {
  auto &cost = Kinodynamic::cost_int2d;
  auto xi = Kinodynamic::sampler();
  auto xf = Kinodynamic::sampler();
  for(auto _ : state)
    cost(xi, xf);
}
BENCHMARK(cost);

static void grow_dynamic_env(benchmark::State &state) {
  auto &rrt = Kinodynamic::rrtstar_int2d_timespace_obs;
  auto &tree = Kinodynamic::tree_int2d;
  auto &env = Kinodynamic::dynamic_soccer_env;

  env.setRandomObstacles();
  auto xg = Kinodynamic::goal_dynamic_env.randomGoal();
  auto xs = Kinodynamic::sampler();
  rrt.setStart(xs);

  auto n = state.range(0);
  for(auto _ : state) {
    for(size_t i=0; i<n; i++)
      rrt.grow(&xg);
    benchmark::DoNotOptimize(rrt.goalIndex());
  }
  // state.SetBytesProcessed(int64_t(state.iterations()) * int64_t(state.range(0)));
}
BENCHMARK(grow_dynamic_env)->ComputeStatistics("solved",[](const std::vector<double>& s)->double{
  auto sol = 0.0;
  for(const auto v : s)
    sol = (v>=0.0 ? 1.0 : sol);
  return sol;
})->Range(10, 1000);

static void neighbor_dynamic_env(benchmark::State &state) {
  auto &rrt = Kinodynamic::rrtstar_int2d_timespace_obs;
  auto &tree = Kinodynamic::tree_int2d;
  auto &env = Kinodynamic::dynamic_soccer_env;
  auto &radius = Kinodynamic::radius;

  auto n = state.range(0);
  double scale = (double(state.range(1))/10.0f);
  radius.scale = scale;

  env.setRandomObstacles();
  auto xg = Kinodynamic::goal_dynamic_env.randomGoal();
  auto xs = Kinodynamic::sampler();
  rrt.setStart(xs);

  for(auto _ : state) {
    for(size_t i=0; i<n; i++)
      rrt.grow(&xg);
    benchmark::DoNotOptimize(rrt.goalIndex());
  }
}

static void bm_neighbor_args(benchmark::internal::Benchmark *b) {
//  for(size_t i=100; i<=1000; i+=100)
//    for(size_t j=11; j<=151; j+=10)
//      b->Args({i,j});
  for(size_t i=11;  i<=TEST_NEIGHBOR; i+=10)
    b->Arg(i);
}

static void bm_nodes_arg(benchmark::internal::Benchmark *b) {
  for(size_t i=100; i<=TEST_NODES; i+=100)
    b->Arg(i);
}

BENCHMARK(neighbor_dynamic_env)->ComputeStatistics("solved",[](const std::vector<double>& s)->double{
  auto sol = 0.0;
  for(const auto v : s)
    sol = (v>=0.0 ? 1.0 : sol);
  return sol;
})->Apply(bm_neighbor_args);

BENCHMARK(neighbor_dynamic_env)->ComputeStatistics("solved",[](const std::vector<double>& s)->double{
  auto sol = 0.0;
  for(const auto v : s)
    sol = (v>=0.0 ? 1.0 : sol);
  return sol;
})->Apply(bm_nodes_arg);

BENCHMARK_MAIN();
