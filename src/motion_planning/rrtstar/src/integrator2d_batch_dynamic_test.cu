#include <boost/filesystem.hpp>
#include "integrator2drrt.cuhpp"
#include "util.h"

#ifndef NDEBUG
#define TRACE_EXEC
#endif

int main(int argc, char** argv)
{
  // enable nan exception
  // feenableexcept(FE_DIVBYZERO);

  ros::init(argc, argv, "rrtstar_integrator2d_test");
  ros::NodeHandle node;

  Models::init_integrator2d();
  RRTVisual vis(node, "test");

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
  // auto &rrt = Kinodynamic::rrtstar_int2d_timespace_obs;
  auto &rrt = Kinodynamic::rrtstar_batch_int2d_timespace_obs;
  auto &tree = Kinodynamic::tree_int2d;
  // auto &env = Kinodynamic::dynamic_soccer_env;
  auto &env = Kinodynamic::dynamic_soccer_env_cuda;

  env.setRandomObstacles();
  auto xg = Kinodynamic::goal_dynamic_env.randomGoal();
  auto xs = Kinodynamic::sampler();
  rrt.setStart(xs);
  // rrt.setIteration(0);

  auto vis_t0 = ros::Time::now();
  bool solved = false;

  while(ros::ok()) {
    auto t0 = ros::Time::now();
    ROS_INFO("growing tree..");
    solved = rrt.grow(&xg);
    auto tree_size = tree.tree.size();
    ROS_INFO("tree size : %d;", tree_size);
    auto t1 = ros::Time::now();

    auto vis_t1 = ros::Time::now();
    auto dt = vis_t1-vis_t0;
    ROS_INFO("dt : %f", dt.toSec());
    if(dt.toSec() > 0.5) {
      ROS_INFO("adding visual..");
      double tf = 10.0;
      double delta = 0.2;
      auto iter = tf/delta;
      auto r = env.collision_radius;
#ifdef TRACE_EXEC
      TRACE_FN("adding circles");
#endif
      for(size_t i=0; i<iter; i++) {
        // draw dynamic obstacles, with black(0.0,0.0,0.0) color,
        // and decreasing opacity over time
        vis.add_circles(env.at(i*delta),r,delta,(i*delta),0.0,0.0,0.0,(iter-i)/iter,"_obstacles");
      }
#ifdef TRACE_EXEC
      DEBUG_PRINT("adding circles", "done");
#endif
      // draw 3d trajectory : xy pos (index 0,1) in xy-plane and time (index 4) as z-plane
      // with green color (0.0,1.0,0.0) and 0.1 opacity
#ifdef TRACE_EXEC
      TRACE_FN("adding trajectories");
#endif
      vis.add_trajectories<3,0,1,4>(tree.trajectories,0.0,1.0,0.0,0.1,"_exploration");
#ifdef TRACE_EXEC
      DEBUG_PRINT("adding trajectories", "done");
#endif
      if(rrt.goalIndex() > 0) {
#ifdef TRACE_EXEC
      TRACE_FN("adding goal");
#endif
        auto goal = tree.get_trajectory(rrt.goalIndex());
        vis.add_trajectories<3,0,1,4>(goal,1.0,1.0,1.0,1.0,"_goal");
#ifdef TRACE_EXEC
      DEBUG_PRINT("adding goal","ok");
#endif
      }
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
