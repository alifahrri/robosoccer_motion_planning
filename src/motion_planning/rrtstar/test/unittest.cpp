#include <gtest/gtest.h>
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

TEST(TimeSolver,Solve) {
  auto &time_diff = Models::integrator2d_opt_time_diff;
  auto &time_solver = Models::integrator2d_opt_time_solver;
  state_t s0, s1;
  s0(0) = 1.54; s0(1) = 0.153; s0(2) = 0.72; s0(3) = 0.15;
  s1(0) = 0.0; s1(1) = -1.2; s1(2) = -0.1; s1(3) = 0.0;

  auto opt_time = time_solver.solve(s0, s1);
  time_diff.set(s0,s1);
  auto d_cost = time_diff(opt_time);
  EXPECT_NEAR(d_cost, 0.0, 0.0001) << d_cost;
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
