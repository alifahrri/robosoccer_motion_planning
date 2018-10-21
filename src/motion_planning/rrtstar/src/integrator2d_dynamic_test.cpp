#include <fenv.h>
#include <boost/filesystem.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include "integrator2drrt.hpp"

int main(int argc, char** argv)
{
  // enable nan exception
  // feenableexcept(FE_DIVBYZERO);

  ros::init(argc, argv, "rrtstar_integrator2d_test");
  ros::NodeHandle node;

  Models::init_integrator2d();
  RRTVisual vis(node, "_test");

  ros::Rate rate(30.0f);
  ros::Duration duration(0,1);

  std::string home;
  ros::get_environment_variable(home, "HOME");

  int target_tree_size = 5000;
  double neighbor_radius_scale = 1.0;
  if(!node.getParam("/target_tree_size",target_tree_size))
    target_tree_size = 5000;
  if(node.getParam("/neighbor_radius_scale",neighbor_radius_scale))
    Kinodynamic::radius.scale = neighbor_radius_scale;

  // auto &rrt = Kinodynamic::rrtstar_int2d;
  auto &rrt = Kinodynamic::rrtstar_int2d_timespace_obs;
  auto &tree = Kinodynamic::tree_int2d;
  auto &env = Kinodynamic::dynamic_soccer_env;
  auto &checker = Kinodynamic::checker_time_space;
  auto &sampler = Kinodynamic::sampler_dynamic_env;
  auto &goal = Kinodynamic::goal_dynamic_env;
  auto &connector = Kinodynamic::connector;

  env.setRandomObstacles();
  auto xg = goal.randomGoal();
  auto xs = sampler();
  rrt.setStart(xs);
  // rrt.setIteration(0);

  auto vis_t0 = ros::Time::now();
  bool solved = false;

  while(ros::ok()) {
    ROS_INFO("growing tree..");
    auto t0 = ros::Time::now();
    solved = rrt.grow(&xg);
    auto tree_size = tree.tree.size();
    auto t1 = ros::Time::now();
    ROS_INFO("tree size : %d;", tree_size);

    auto vis_t1 = ros::Time::now();
    auto dt = vis_t1-vis_t0;
    ROS_INFO("dt : %f", dt.toSec());
    if(dt.toSec() > 0.5) {
      ROS_INFO("adding visual..");
      double tf = 10.0;
      double delta = 0.2;
      auto iter = tf/delta;
      auto r = env.collision_radius;
      for(size_t i=0; i<iter; i++) {
        // draw dynamic obstacles, with black(0.0,0.0,0.0) color,
        // and decreasing opacity over time
        vis.add_circles(env.at(i*delta),r,delta,(i*delta),0.0,0.0,0.0,(iter-i)/iter,"_obstacles");
      }
      // draw 3d trajectory : xy pos (index 0,1) in xy-plane and time (index 4) as z-plane
      // with green color (0.0,1.0,0.0) and 0.1 opacity
      vis.add_trajectories<3,0,1,4>(tree.trajectories,0.0,1.0,0.0,0.1,"_exploration");
      if(rrt.goalIndex() > 0) {
        auto goal = tree.get_trajectory(rrt.goalIndex());
        vis.add_trajectories<3,0,1,4>(goal,1.0,1.0,1.0,1.0,"_goal");
      }
      // draw start and goal in 2D
      vis.add_point<2,0,1>(xs, 1.0f, 0.0f, 0.0f, 1.0f, "_start");
      vis.add_point<2,0,1>(xg, 0.0f, 0.0f, 1.0f, 1.0f, "_goal");
      vis.add_point<2,0,1>(sampler.last_sample(), 0.0f, 1.0f, 1.0f, 1.0f, "_last_sampled");
      auto last_edge = connector.last_connection();
      vis.add_trajectories<3,0,1,4>(std::vector<decltype(last_edge)>{last_edge}, 1.0f, 0.0f, 0.0f, 1.0f, "_last_connection");
      /*
      for(size_t i=0; i<5; i++) {
        auto s = tree(0);
        auto xr = sampler();
        auto c = connector(s,xr);
        {
          auto r = 1.0f; auto g = 1.0f; auto b = 0.0f;
          if(checker(c)) g = 0.0f;
          vis.add_trajectories<3,0,1,4>(std::vector<decltype(c)>{c}, r, g, b, 1.0f, "_test");
        }
      }
      */
      ROS_INFO("publish visual..");
      // clear all before re-drawing
      vis.delete_all();
      vis.publish();
      vis.clear();
      vis_t0 = vis_t1;
    }

    duration.sleep();
  }

  return 0;
}
