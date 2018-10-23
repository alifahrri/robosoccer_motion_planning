#include <gtest/gtest.h>
#include <random>
#include "integrator2drrt.hpp"
#include "random.hpp"
#include "states.hpp"
#include "environment.hpp"

typedef State<double,Models::n> state_t;
typedef RandomGen<2,double> RandomGen2;
typedef RandomGen<4,double> RandomGen4;
typedef Kinodynamic::TreeInt2D TreeInt2D;
typedef Kinodynamic::Connector Connector;
typedef Kinodynamic::CollisionChecker CollisionChecker;
typedef Kinodynamic::CollisionTimeSpaceChecker CollisionTimeSpaceChecker;
typedef Kinodynamic::Sampler<Kinodynamic::StaticEnvironment> Sampler;
typedef Kinodynamic::GoalChecker<Kinodynamic::StaticEnvironment> GoalChecker;
typedef Robosoccer<> Environment;
typedef Models::Integrator2DOptTimeDiff TimeDiff;
typedef Models::Integrator2DOptTimeSolver TimeSolver;
typedef Models::Integrator2DTrajectorySolver TrajectorySolver;
typedef Models::Integrator2DSS Integrator2DSS;
typedef Models::Integrator2DGramian Integrator2DGramian;
typedef Models::Integrator2DClosedExpm Integrator2DClosedExpm;
typedef Models::Integrator2DSSComposite Integrator2DSSComposite;
typedef Models::Integrator2DCmpClosedExpm Integrator2DCmpClosedExpm;
typedef Models::Integrator2DSSComposite::StateType Integrator2DSSCompositeState;
typedef Models::Integrator2DSSComposite::SystemMatrix Integrator2DSSCompositeSystem;

TEST(TimeSolver,Solve) {
  auto &time_diff = Models::integrator2d_opt_time_diff;
  auto &time_solver = Models::integrator2d_opt_time_solver;
  auto &sampler = Kinodynamic::sampler;
  auto s0 = sampler();
  auto s1 = sampler();

  auto opt_time = time_solver.solve(s0, s1);
  time_diff.set(s0,s1);
  auto d_cost = time_diff(opt_time);
  EXPECT_NEAR(d_cost, 0.0, 0.0001) << d_cost;
}

TEST(TreeInt2D,insert)
{
  auto &tree = Kinodynamic::tree_int2d;
  auto &sampler = Kinodynamic::sampler;
  auto &connector = Kinodynamic::connector;
  auto n = 100;
  auto s = sampler();
  tree.reset();
  tree.insert(s, -1);
  auto ok = true;
  for(size_t i=0; i<n; i++) {
    auto ts = tree.tree.size();
    auto ps = tree.parent.size();
    auto es = tree.trajectories.size();
    if(((ts-1)!=i) || ((ps-1)!=i) || ((es-1)!=i)) ok = false;
    auto s = sampler();
    auto e = connector(tree(ts-1), s);
    tree.insert(s, tree.tree.size()-1, e);
  }
  EXPECT_TRUE(ok);
}

TEST(CollisionChecker, collision) {
  auto &checker = Kinodynamic::checker;
  auto &connector = Kinodynamic::connector;
  auto &sampler = Kinodynamic::sampler;
  auto &env = Kinodynamic::robosoccer_env;

  using state_t = decltype(sampler());
  state_t s0, s1;

  std::array<state_t,9> obs;
  env.setObstacles(obs);

  s0(0) = -5.0; s1(0) = 5.0;
  s0(1) = 0.0; s1(1) = 0.0;
  s0(2) = 0.0; s1(2) = 0.0;
  s0(3) = 0.0; s1(3) = 0.0;
  auto e = connector(s0, s1);
  EXPECT_TRUE(checker(e)) << s0 << s1;
}

TEST(CollisionChecker, collision_free) {
  auto &checker = Kinodynamic::checker;
  auto &connector = Kinodynamic::connector;
  auto &sampler = Kinodynamic::sampler;
  auto &env = Kinodynamic::robosoccer_env;

  using state_t = decltype(sampler());
  state_t s0, s1;

  std::array<state_t,9> obs;
  env.setObstacles(obs);

  s0(0) = -5.0; s1(0) = -5.0;
  s0(1) = -5.0; s1(1) = 5.0;
  s0(2) = 0.0; s1(2) = 0.0;
  s0(3) = 0.0; s1(3) = 0.0;
  auto e = connector(s0, s1);
  EXPECT_FALSE(checker(e)) << s0 << s1;
}

TEST(CollisionTimeSpaceChecker, collision) {
  auto &checker = Kinodynamic::checker;
  auto &connector = Kinodynamic::connector;
  auto &sampler = Kinodynamic::sampler;
  auto &env = Kinodynamic::robosoccer_env;

  using state_t = decltype(sampler());
  state_t s0, s1;

  std::array<state_t,9> obs;
  env.setObstacles(obs);

  s0(0) = -5.0; s1(0) = 5.0;
  s0(1) = 0.0; s1(1) = 0.0;
  s0(2) = 0.0; s1(2) = 0.0;
  s0(3) = 0.0; s1(3) = 0.0;
  auto e = connector(s0, s1);
  EXPECT_TRUE(checker(e)) << s0 << s1;
}

TEST(CollisionTimeSpaceChecker, collision_free) {
  auto &checker = Kinodynamic::checker_time_space;
  auto &connector = Kinodynamic::connector;
  auto &sampler = Kinodynamic::sampler_dynamic_env;
  auto &env = Kinodynamic::dynamic_soccer_env;

  using state_t = decltype(sampler());
  state_t s0, s1;

  std::array<state_t,9> obs;
  env.setObstacles(obs);

  s0(0) = -5.0; s1(0) = -5.0;
  s0(1) = -5.0; s1(1) = 5.0;
  s0(2) = 0.0; s1(2) = 0.0;
  s0(3) = 0.0; s1(3) = 0.0;
  auto e = connector(s0, s1);
  EXPECT_FALSE(checker(e)) << s0 << s1;
}

using RRT = decltype(Kinodynamic::rrtstar_int2d_timespace_obs);

TEST(RRT,grow)
{
  auto &rrt = Kinodynamic::rrtstar_int2d_timespace_obs;
  auto &tree = Kinodynamic::tree_int2d;
  auto &env = Kinodynamic::dynamic_soccer_env;
  auto &checker = Kinodynamic::checker_time_space;
  auto &sampler = Kinodynamic::sampler_dynamic_env;
  auto &goal = Kinodynamic::goal_dynamic_env;
  auto &connector = Kinodynamic::connector;
  auto min = sampler.rg->min();
  auto max = sampler.rg->max();

  env.setRandomObstacles();
  auto xg = goal.randomGoal();
  auto xs = sampler();
  rrt.setStart(xs);
  auto solved = false;
  auto n = 100;

  std::stringstream ss;
  auto ok = true;
  for(size_t i=0; i<n; i++) {
    auto ts = tree.tree.size();
    auto ps = tree.parent.size();
    auto es = tree.trajectories.size();
    solved = rrt.grow(&xg);
    auto tree_size = tree.tree.size();
    // check if tree start is right
    if(tree(0)!=xs)
    {
      ss << "start state changed; ";
      ok = false;
    }
  }
  {
    auto ts = tree.tree.size();
    auto ps = tree.parent.size();
    auto es = tree.trajectories.size();
    // check if tree, parent, and trajectory size matched
    if((ts != ps) || (ps != es) || (ts != es))
    {
      ss << "tree (or parent or trajectory) has different size; ";
      ok = false;
    }
    if(ok)
      for(size_t i=1; i<ts; i++) {
        auto s0_t = tree.trajectories.at(i).path().front();
        auto s1_t = tree.trajectories.at(i).path().back();
        auto s0_p = tree.tree(tree.parent.at(i));
        auto s1_p = tree.tree(i);
        auto printer = [](decltype(s0_t) s){
          std::stringstream ss;
          ss << "[";
          for(size_t i=0; i<4; i++)
            ss << s(i) << (i==3 ? "] " : " ");
          return ss.str();
        };
        auto near = [](decltype(s0_t) t, decltype(s0_t) p, double e, double *err, int *id)
        {
          auto n = true;
          for(size_t i=0; i<4; i++) {
            *err = fabs(t(i) - p(i));
            if(*err > e) {
              n = false;
              *id = i;
              break;
            }
          }
          return n;
        };
        auto in_range = [](decltype(s0_t) s, decltype(min) min, decltype(max) max)
        {
          auto ok = true;
          for(size_t i=0; i<4; i++) {
            if((s(i) < min[i]) || s(i) > max[i]) {
              ok = false;
              break;
            }
          }
          return ok;
        };
        double s0_err, s1_err;
        int id0 = 0, id1 = 0;
        // check if tree, parent, and trajectory is consistent
        if(!near(s0_t, s0_p, 1e-3, &s0_err, &id0) || !near(s1_t, s1_p, 1e-3, &s1_err, &id1))
        {
          ss << "tree nodes didn't match: "
             << "front : "<< printer(s0_t)
             << "parent : " << printer(s0_p)
             << "err " <<  s0_err << " "
             << "at : " << id0 << "; "
             << "back : " << printer(s1_t)
             << "node : " << printer(s1_p)
             << "err " <<  s1_err
             << "at : " << id1 << "; ";
          ok = false;
          break;
        }
        if(!in_range(s1_p, min, max)) {
          ss << "some value are outside sampler range!";
          ok = false;
          break;
        }
      }
  }
  EXPECT_TRUE(ok) << ss.str();
}

TEST(TreeInt2D,retrieve)
{
  auto &tree = Kinodynamic::tree_int2d;
  auto &sampler = Kinodynamic::sampler;
  auto &connector = Kinodynamic::connector;
  auto n = 100;
  auto s = sampler();
  auto ls = s;
  tree.reset();
  tree.insert(s, -1);
  auto ok = true;
  for(size_t i=0; i<n; i++) {
    auto ts = tree.tree.size();
    auto ps = tree.parent.size();
    auto es = tree.trajectories.size();
    if(tree(i)!=ls) ok = false;
    auto s = sampler();
    auto e = connector(tree(ts-1), s);
    tree.insert(s, tree.tree.size()-1, e);
    ls = s;
  }
  EXPECT_TRUE(ok);
}

TEST(TreeInt2D,parent)
{
  auto &tree = Kinodynamic::tree_int2d;
  auto &sampler = Kinodynamic::sampler;
  auto &connector = Kinodynamic::connector;
  auto n = 100;
  auto s = sampler();
  tree.reset();
  tree.insert(s, -1);
  auto lp = -1;
  auto ok = true;
  for(size_t i=0; i<n; i++) {
    auto ts = tree.tree.size();
    auto ps = tree.parent.size();
    auto es = tree.trajectories.size();
    if(i>0)
      if(tree.parent.at(i-1) != lp) ok = false;
    auto s = sampler();
    auto e = connector(tree(ts-1), s);
    auto max = ts-1;
    std::random_device rd;     // only used once to initialise (seed) engine
    std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
    std::uniform_int_distribution<int> uni(0,max); // guaranteed unbiased
    auto random_integer = uni(rng);
    lp = random_integer;
    tree.setParent(i, lp);
    tree.insert(s, tree.tree.size()-1, e);
  }
  EXPECT_TRUE(ok);
}

TEST(TreeInt2D,edge)
{
  auto &tree = Kinodynamic::tree_int2d;
  auto &sampler = Kinodynamic::sampler;
  auto &connector = Kinodynamic::connector;
  auto n = 100;
  auto s = sampler();
  auto le = connector(s, sampler());
  tree.reset();
  tree.insert(s, -1, le);
  auto ok = true;
  std::vector<int> failed;
  if(tree.trajectories.back() != le)
  {
    failed.push_back(0);
    ok = false;
  }
  if(ok)
    for(size_t i=0; i<n; i++) {
      auto ts = tree.tree.size();
      auto ps = tree.parent.size();
      auto es = tree.trajectories.size();
      if(tree.trajectories.at(i) != le)
      {
        failed.push_back(i);
        ok = false;
      }
      auto s = sampler();
      auto e = connector(tree(ts-1), s);
      tree.insert(s, ts-1, e);
      auto s0 = sampler();
      auto s1 = sampler();
      e = connector(s0, s1);
      tree.setEdge(ts, e);
      le = e;
    }
  std::stringstream ss;
  ss << "failed test : ";
  for(auto i : failed)
    ss << i << " ";
  EXPECT_TRUE(ok) << ss.str();
}

TEST(Connector, Solve)
{
  auto &connector = Kinodynamic::connector;
  auto &sampler = Kinodynamic::sampler;
  auto &tree = Kinodynamic::tree_int2d;
  tree.reset();
  auto s0 = sampler();
  auto s1 = sampler();
  auto connection = connector(s0,s1);

  std::stringstream ss;
  ss << "("
     << s0(0) << ","
     << s0(1) << ","
     << s0(2) << ","
     << s0(3)
     << ")->("
     << s1(0) << ","
     << s1(1) << ","
     << s1(2) << ","
     << s1(3)
     << ")" << std::endl;
  // check for inf or nan
  auto ok = true;
  for(const auto t : connection) {
    ss << "("
       << t(0) << ","
       << t(1) << ","
       << t(2) << ","
       << t(3) << ","
       << t(4) << ")"
       << std::endl;
    for(size_t i=0; i<=4; i++)
      if(isnan(t(i)) || isinf(t(i)))
        ok = false;
  }
  for(const auto t : connector.last_connection()) {
    ss << "("
       << t(0) << ","
       << t(1) << ","
       << t(2) << ","
       << t(3) << ","
       << t(4) << ")"
       << std::endl;
    for(size_t i=0; i<=4; i++)
      if(isnan(t(i)) || isinf(t(i)))
        ok = false;
  }
  EXPECT_TRUE(ok) << ss.str();
}

TEST(TrajectorySolver, Solve)
{
  auto &solver = Models::integrator2d_trj_solver;
  auto &sampler = Kinodynamic::sampler;
  auto s0 = sampler();
  auto s1 = sampler();
  auto trj = solver.solve(s0,s1);
  auto ok = true;
  std::stringstream ss;
  ss << "("
     << s0(0) << ","
     << s0(1) << ","
     << s0(2) << ","
     << s0(3)
     << ")->("
     << s1(0) << ","
     << s1(1) << ","
     << s1(2) << ","
     << s1(3)
     << ")" << std::endl;
  for(const auto &t : trj) {
    auto time = std::get<0>(t);
    auto state = std::get<1>(t);
    auto input = std::get<2>(t);
    ss << "("
       << state(0) << ","
       << state(1) << ","
       << state(2) << ","
       << state(3) << ","
       << time << ")"
       << std::endl;
    if(isnan(time) || isinf(time)) ok = false;
    for(size_t i=0; i<4; i++)
      if(isnan(state(i)) || isinf(state(i)))
        ok = false;
  }
  EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator2DClosedExpm, exp)
{
  Integrator2DClosedExpm int2d_exp;
  auto ok = true;
  std::stringstream ss;
  for(size_t i=0; i<10; i++) {
    auto t = i*0.5;
    auto m = int2d_exp(t);
    ss << "t(" << t << ") : [";
    for(size_t j=0; j<4; j++) {
      for(size_t k=0; k<4; k++)
      {
        ss << m(j,k) << (k!=3 ? " " : "; ");
        if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
      }
    }
    ss << "]" << std::endl;
  }
  EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator2DSS, exp)
{
  // Integrator2DSS int2d;
  auto &int2d = Models::integrator2d;
  auto ok = true;
  std::stringstream ss;
  for(size_t i=0; i<10; i++) {
    auto t = i*0.5;
    auto m = int2d.expm(t);
    ss << "t(" << t << ") : [";
    for(size_t j=0; j<4; j++) {
      for(size_t k=0; k<4; k++)
      {
        ss << m(j,k) << (k!=3 ? " " : "; ");
        if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
      }
    }
    ss << "]" << std::endl;
  }
  EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator2DSSComposite, exp)
{
  // Integrator2DSSComposite ss_int2d;
  auto &ss_int2d = Models::integrator2d_ss_cmp;
  auto ok = true;
  std::stringstream ss;
  for(size_t i=0; i<10; i++) {
    auto t = i*0.5;
    auto m = ss_int2d.expm(t);
    ss << "t(" << t << ") : [";
    for(size_t j=0; j<4; j++) {
      for(size_t k=0; k<4; k++)
      {
        ss << m(j,k) << (k!=3 ? " " : "; ");
        if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
      }
    }
    ss << "]" << std::endl;
  }
  EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator2DGramian, gram)
{
  Integrator2DGramian g;
  auto ok = true;
  std::stringstream ss;
  for(size_t i=1; i<30; i++) {
    auto t = i*0.5;
    auto m = g(t);
    auto m_inv = m.inverse();
    ss << "t(" << t << ") : [";
    for(size_t j=0; j<4; j++) {
      for(size_t k=0; k<4; k++)
      {
        ss << m(j,k) << (k!=3 ? " " : "; ");
        if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
      }
    }
    ss << "]" << std::endl;
    ss << "t(" << t << ") : inverse [";
    for(size_t j=0; j<4; j++) {
      for(size_t k=0; k<4; k++)
      {
        ss << m_inv(j,k) << (k!=3 ? " " : "; ");
        if(isnan(m_inv(j,k)) || isinf(m_inv(j,k))) ok = false;
      }
    }
    ss << "]" << std::endl;
  }
  EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator2DGramian, d_opt)
{
  auto &g = Models::integrator2d_gram;
  auto &tsolver = Models::integrator2d_opt_time_solver;
  auto &sampler = Kinodynamic::sampler;
  auto &int2d = Models::integrator2d;
  auto s0 = sampler();
  auto s1 = sampler();
  auto opt_time = tsolver.solve(s0, s1);
  auto g_mat = g(opt_time);
  auto g_inv = g_mat.inverse();
  auto d_opt = g_inv*(s1-int2d.expm((opt_time))*s0);

  auto ok = true;
  std::stringstream ss;
  ss << "("
     << s0(0) << ","
     << s0(1) << ","
     << s0(2) << ","
     << s0(3)
     << ")->("
     << s1(0) << ","
     << s1(1) << ","
     << s1(2) << ","
     << s1(3)
     << ")" << std::endl;
  ss << "opt_time(" << opt_time << ") : G : [";
  for(size_t j=0; j<4; j++)
  {
    for(size_t k=0; k<4; k++)
    {
      ss << g_mat(j,k) << (k!=3 ? " " : "; ");
      if(isnan(g_mat(j,k)) || isinf(g_mat(j,k))) ok = false;
    }
  }
  ss << "]" << std::endl;
  ss << "opt_time(" << opt_time << ") : g_inv : [";
  for(size_t j=0; j<4; j++)
  {
    for(size_t k=0; k<4; k++)
    {
      ss << g_inv(j,k) << (k!=3 ? " " : "; ");
      if(isnan(g_inv(j,k)) || isinf(g_inv(j,k))) ok = false;
    }
  }
  ss << "]" << std::endl;
  ss << "opt_time(" << opt_time << ") : [";
  for(size_t k=0; k<4; k++)
  {
    ss << d_opt(k) << (k==3 ? "" : " ");
    if(isnan(d_opt(k)) || isinf(d_opt(k))) ok = false;
  }
  ss << "]" << std::endl;
  EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator2DCmpClosedExpm, exp)
{
  Integrator2DCmpClosedExpm cmp_int2d_exp;
  auto ok = true;
  std::stringstream ss;
  for(size_t i=0; i<10; i++) {
    auto t = i*0.5;
    auto m = cmp_int2d_exp(t);
    ss << "t(" << t << ") : [";
    for(size_t j=0; j<8; j++) {
      for(size_t k=0; k<8; k++)
      {
        ss << m(j,k) << (k==7 ? " " : "; ");
        if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
      }
    }
    ss << "]" << std::endl;
  }
  EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator2DSSCompositeState, composite_state)
{
  auto &g = Models::integrator2d_gram;
  auto &tsolver = Models::integrator2d_opt_time_solver;
  auto &sampler = Kinodynamic::sampler;
  auto &int2d = Models::integrator2d;
  auto s0 = sampler();
  auto s1 = sampler();
  auto opt_time = tsolver.solve(s0, s1);
  auto d_opt = g(opt_time).inverse()*(s1-int2d.expm((opt_time))*s0);
  Integrator2DSSCompositeState cmp_state;
  cmp_state << s1, d_opt;

  auto ok = true;
  std::stringstream ss;
  ss << "("
     << s0(0) << ","
     << s0(1) << ","
     << s0(2) << ","
     << s0(3)
     << ")->("
     << s1(0) << ","
     << s1(1) << ","
     << s1(2) << ","
     << s1(3)
     << ")" << std::endl;
  ss << "[ ";
  ss << "opt_time(" << opt_time << ") : [";
  for(size_t k=0; k<4; k++)
  {
    ss << d_opt(k) << (k==3 ? "" : " ");
    if(isnan(d_opt(k)) || isinf(d_opt(k))) ok = false;
  }
  ss << "]" << std::endl;
  for(size_t j=0; j<8; j++) {
    ss << cmp_state(j) << (j==7 ? "" : " ");
    if(isnan(cmp_state(j)) || isinf(cmp_state(j))) ok = false;
  }
  ss << "]" << std::endl;
  EXPECT_TRUE(ok) << ss.str();
}

TEST(RandomGen2, Random) {
  state_t s0, s1;
  RandomGen2 *rg = new RandomGen2({-SAMPLE_X0, -SAMPLE_X1, -SAMPLE_X2, -SAMPLE_X3},{SAMPLE_X0, SAMPLE_X1, SAMPLE_X2, SAMPLE_X3});
  std::stringstream ss;
  (*rg)(s0); (*rg)(s1);
  ss << "s0(" << s0(0) << "," << s0(1) << "," << s0(2) << "," << s0(3) << ") "
     << "s1(" << s1(0) << "," << s1(1) << "," << s1(2) << "," << s1(3) << ")";
  // EXPECT_TRUE(false) << ss.str();
  // ASSERT_NE(s0, s1) << ss.str();
  auto min = rg->min();
  auto max = rg->max();
  EXPECT_TRUE((s0(0)!=s1(0)) && (s0(0) >= min[0]) && (s1(0) >= min[0]) && (s0(0) <= max[0]) && (s1(0) <= max[0]) &&
      (s0(1)!=s1(1)) && (s0(1) >= min[1]) && (s1(1) >= min[1]) && (s0(1) <= max[1]) && (s1(1) <= max[1])) << ss.str();
}

TEST(RandomGen4, Random) {
  state_t s0, s1;
  RandomGen4 *rg = new RandomGen4({-SAMPLE_X0, -SAMPLE_X1, -SAMPLE_X2, -SAMPLE_X3},{SAMPLE_X0, SAMPLE_X1, SAMPLE_X2, SAMPLE_X3});
  std::stringstream ss;
  (*rg)(s0); (*rg)(s1);
  ss << "s0(" << s0(0) << "," << s0(1) << "," << s0(2) << "," << s0(3) << ") "
     << "s1(" << s1(0) << "," << s1(1) << "," << s1(2) << "," << s1(3) << ")";
  // EXPECT_TRUE(false) << ss.str();
  // ASSERT_NE(s0, s1) << ss.str();
  auto min = rg->min();
  auto max = rg->max();
  EXPECT_TRUE((s0(0)!=s1(0)) && (s0(0) >= min[0]) && (s1(0) >= min[0]) && (s0(0) <= max[0]) && (s1(0) <= max[0]) &&
      (s0(1)!=s1(1)) && (s0(1) >= min[1]) && (s1(1) >= min[1]) && (s0(1) <= max[1]) && (s1(1) <= max[1]) &&
      (s0(2)!=s1(2)) && (s0(2) >= min[2]) && (s1(2) >= min[2]) && (s0(2) <= max[2]) && (s1(2) <= max[2]) &&
      (s0(3)!=s1(3)) && (s0(3) >= min[3]) && (s1(3) >= min[3]) && (s0(3) <= max[3]) && (s1(3) <= max[3])) << ss.str();
}

TEST(Sampler, Sample) {
  state_t s0 = Kinodynamic::sampler();
  state_t s1 = Kinodynamic::sampler();
  std::stringstream ss;
  ss << "s0(" << s0(0) << "," << s0(1) << "," << s0(2) << "," << s0(3) << ") "
     << "s1(" << s1(0) << "," << s1(1) << "," << s1(2) << "," << s1(3) << ")";
  auto min = Kinodynamic::sampler.rg->min();
  auto max = Kinodynamic::sampler.rg->max();
  EXPECT_TRUE((s0(0)!=s1(0)) && (s0(0) >= min[0]) && (s1(0) >= min[0]) && (s0(0) <= max[0]) && (s1(0) <= max[0]) &&
      (s0(1)!=s1(1)) && (s0(1) >= min[1]) && (s1(1) >= min[1]) && (s0(1) <= max[1]) && (s1(1) <= max[1]) &&
      (s0(2)!=s1(2)) && (s0(2) >= min[2]) && (s1(2) >= min[2]) && (s0(2) <= max[2]) && (s1(2) <= max[2]) &&
      (s0(3)!=s1(3)) && (s0(3) >= min[3]) && (s1(3) >= min[3]) && (s0(3) <= max[3]) && (s1(3) <= max[3])) << ss.str();
}

TEST(GoalChecker, RandomGoal) {
  auto s0 = Kinodynamic::goal.randomGoal();
  auto s1 = Kinodynamic::goal.randomGoal();
  std::stringstream ss;
  auto min = Kinodynamic::goal.rg->min();
  auto max = Kinodynamic::goal.rg->max();
  EXPECT_TRUE((s0(0)!=s1(0)) && (s0(0) >= min[0]) && (s1(0) >= min[0]) && (s0(0) <= max[0]) && (s1(0) <= max[0]) &&
      (s0(1)!=s1(1)) && (s0(1) >= min[1]) && (s1(1) >= min[1]) && (s0(1) <= max[1]) && (s1(1) <= max[1])) << ss.str();
}

TEST(Environment, Random) {
  auto &env = Kinodynamic::robosoccer_env;
  env.setRandomObstacles();
  auto o1 = env.obs;
  env.setRandomObstacles();
  auto o2 = env.obs;
  auto ne = true;
  std::stringstream ss1;
  std::stringstream ss2;
  auto min = env.rg->min();
  auto max = env.rg->max();
  for(size_t i=0; i<o1.size(); i++) {
    if(o1[i] == o2[i]) ne = false;
    if((std::get<0>(o1[i]) < min[0]) || (std::get<1>(o1[i]) < min[1]) ||
       (std::get<0>(o1[i]) > max[0]) || (std::get<1>(o1[i]) > max[1]) ||
       (std::get<0>(o2[i]) < min[0]) || (std::get<1>(o2[i]) < min[1]) ||
       (std::get<0>(o2[i]) > max[0]) || (std::get<1>(o2[i]) > max[1]))
      ne = false;
    ss1 << "(" << std::get<0>(o1[i]) << "," << std::get<1>(o1[i]) << ") ";
    ss2 << "(" << std::get<0>(o2[i]) << "," << std::get<1>(o2[i]) << ") ";
  }
  // ASSERT_NE(o1, o2);
  EXPECT_TRUE(ne) << "o1 : " << ss1.str() << "o2 : " << ss2.str();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc,argv);
  Models::init_integrator2d();
  return RUN_ALL_TESTS();
}
