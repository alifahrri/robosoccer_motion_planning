#ifndef ROBOSOCCER_VISUAL_HPP
#define ROBOSOCCER_VISUAL_HPP

#include <ros/ros.h>
#include "integrator2drrt.hpp"
#include "rrtvisual.hpp"

template<typename RRT, typename Tree, typename Env>
inline
auto call(RRT &rrt, Tree &tree, Env &env, RRTVisual &vis)
{
  // c++17 stuff (compile-time if)
  if constexpr(std::is_same<Env, decltype(Kinodynamic::robosoccer_env)>::value) {
    ROS_INFO("adding visual..");
    auto r = env.collision_radius;
    for(const auto &o : env.obs)
      vis.add_obstacles(std::get<0>(o),std::get<1>(o),r);
    vis.set_trajectories(tree.tree.cloud.states, tree.trajectories, tree.parent, 2, tree.tree.size());
    if(rrt.goalIndex() > 0) {
      auto goal_trj = tree.get_trajectory(rrt.goalIndex());
      for(const auto &t : goal_trj)
        vis.set_trajectory(t.path(),2);
    }
    ROS_INFO("publish visual..");
  }
  if constexpr(std::is_same<Env, decltype(Kinodynamic::dynamic_soccer_env)>::value) {
    ROS_INFO("adding visual (dynamic_env)..");
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
    auto xs = tree(0);
    auto xg = (rrt.goalIndex() > 0 ? tree(rrt.goalIndex()) : xs);
    // draw start and goal in 2D
    vis.add_point<2,0,1>(xs, 1.0f, 0.0f, 0.0f, 1.0f, "_start");
    vis.add_point<2,0,1>(xg, 0.0f, 0.0f, 1.0f, 1.0f, "_goal");
    ROS_INFO("publish visual..");
  }
}

#endif // ROBOSOCCER_VISUAL_HPP
