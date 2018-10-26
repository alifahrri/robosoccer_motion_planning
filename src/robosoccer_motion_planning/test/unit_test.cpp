#include <gtest/gtest.h>
#include <ros/ros.h>
#include "trajectory1d.h"

typedef Trajectory1D::AngleController AngleTrajectoryGenerator;

TEST(AngleTrajectoryGenerator, angle_shortest_path)
{
  auto trj = AngleTrajectoryGenerator();
  auto offset = 0.0;
  auto t_offset = 0.0;
  std::vector<double> from{M_PI+0.1, 0.0,  1.32,   -3.0,   1.7,  2.3, -2.3};
  std::vector<double> to  {M_PI,     1.32, 0.0,    1.7,    -3.0, -1.4, 1.4};
  std::vector<double> res {-0.1,     1.32, -1.32,  -1.58,  1.58, 2.58, -2.58};
  auto f_it = from.begin();
  auto t_it = to.begin();
  auto r_it = res.begin();
  auto ok = true;
  std::stringstream ss;
  auto near = [](double a, double b, double e){ return fabs(a-b) < e; };
  for(;(f_it != from.end()) && (t_it != to.end()) && (r_it != res.end()); f_it++, t_it++, r_it++)
  {
    auto d = trj.shortest_path(*f_it, *t_it, offset, t_offset);
    if(!near(d, *r_it, 0.01)) {
      ok = false;
      ss << *f_it << " to : " << *t_it << " results : " << d;
      break;
    }
  }
  EXPECT_TRUE(ok) << ss.str();
}

TEST(AngleTrajectoryGenerator, trajectory)
{
  auto trj = AngleTrajectoryGenerator();
  auto offset = 0.0;
  auto t_offset = 0.0;
  std::vector<double> from{M_PI+0.1, M_PI,      0.0, 1.32, -3.0,   1.7,   2.3, -2.3};
  std::vector<double> to  {M_PI,     M_PI+0.1,  1.32, 0.0,    1.7,   -3.0, -1.4, 1.4};
  std::vector<double> res {-0.1,     0.1,       1.32, -1.32,  -1.58, 1.58, 2.58, -2.58};
  auto f_it = from.begin();
  auto t_it = to.begin();
  auto r_it = res.begin();
  auto ok = true;
  std::stringstream ss;
  auto near = [](double a, double b, double e){ return fabs(a-b) < e; };
  auto norm_angle = [](double a) {
    return fabs(a) > M_PI ? (a - (a < 0? -1 : 1) * M_PI) : a;
  };
  auto angle_dist = [](double a, double b){
    auto d = a-b;
    return (fabs(d) < M_PI ? d : (d < 0 ? 1 : -1) * (2*M_PI - (fabs(a)+fabs(b))));
  };
  for(;(f_it != from.end()) && (t_it != to.end()) && (r_it != res.end()); f_it++, t_it++, r_it++)
  {
    auto time = 0.0;
    auto d = trj.shortest_path(*f_it, *t_it, offset, t_offset);
    auto ctrl = trj.angleControl({*f_it,0.0},*t_it,time);
    auto angle_trj = AngleTrajectoryGenerator::getAngleTrajectory(ctrl, trj.getOffset(), 0.0, time, 20);
    std::stringstream trj_str;
    for(auto &a : angle_trj.first) {
      trj_str << "(" << a.w << "," << a.dw <<") ";
      if(fabs(a.w) > M_PI) {
        ok = false;
        ss << *f_it << " to : " << *t_it << " results : " << a.w;
        break;
      }
    }
    auto e = fabs(angle_dist(angle_trj.first.back().w,*t_it));
    if(e > 0.01) {
      ok = false;
      ss << *f_it << " to : " << *t_it
         << "| results : " << trj_str.str()
         << "| shortest distance : " << d
         << "| ctrl distance : " << ctrl.distance
         << "| opt time : " << time
         << "| total time : " << angle_trj.second.back()
         << "| offset : " << trj.getOffset().f_offset
         << ", " << trj.getOffset().t_offset
         << "| offset_start : " << *f_it + trj.getOffset().f_offset
         << "| e : " << e;
      break;
    }
  }
  EXPECT_TRUE(ok) << ss.str();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
