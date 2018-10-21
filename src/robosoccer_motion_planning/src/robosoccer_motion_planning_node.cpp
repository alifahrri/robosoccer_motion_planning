#include <ros/ros.h>
#include <type_traits>
#include <rrtstar.hpp>
#include "integrator2drrt.hpp"
#include "robotsubscriber.h"
#include "robosoccer_visual.hpp"

enum Environment {
  Static,
  Dynamic
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robosoccer_motion_planning");
  ros::NodeHandle node;
  size_t agent_id = 1;

  RRTVisual vis(node);

  auto subs = RobotSubscriber(node, agent_id, RobotSubscriber::NUBOT);

  Models::init_integrator2d();

  auto &rrt = Kinodynamic::rrtstar_int2d;
  auto &env = Kinodynamic::robosoccer_env;
  auto &goal = Kinodynamic::goal;
  auto &sampler = Kinodynamic::sampler;

  auto &rrt_dyn = Kinodynamic::rrtstar_int2d_timespace_obs;
  auto &env_dyn = Kinodynamic::dynamic_soccer_env;
  auto &goal_dyn = Kinodynamic::goal_dynamic_env;
  auto &sampler_dyn = Kinodynamic::sampler_dynamic_env;

  auto &tree = Kinodynamic::tree_int2d;

  // generic lambdas :
  // set starting state for rrt given the subscriber
  auto set_start = [](auto &rrt, auto &subs) {
    auto s = subs.getState();
    Kinodynamic::TreeInt2D::State xs;
    xs(0) = s(0); xs(1) = s(1);
    xs(2) = s(2); xs(3) = s(3);
    ROS_INFO("xs(%f,%f,%f,%f)",xs(0),xs(1),xs(2),xs(3));
    rrt.setStart(xs);
    rrt.setIteration(0);
  };
  // set obstacles for the environment
  auto set_obstacles = [](auto &env, auto &subs) {
    env.setObstacles(subs.getObstacles());
  };
  // some visualization stuff
  auto visualize = [](auto &rrt, auto &tree, auto &env, auto &vis) {
    call(rrt, tree, env, vis);
    // clear all before re-drawing
    vis.delete_all();
    vis.publish();
    vis.clear();
  };
  // create helper lambda to run the rrt
  auto solve_rrt = [set_start, set_obstacles, visualize, &vis](auto &rrt, auto &subs, auto &env, auto &tree, auto *xg, size_t iteration) {
    set_start(rrt, subs);
    set_obstacles(env, subs);
    auto solved = false;
    auto t0 = ros::Time::now();
    for(size_t i=0; i<iteration; i++)
        solved = rrt.grow(xg);
    auto t1 = ros::Time::now();
    auto dt = t1 - t0;
    if(tree.tree.size() > 0) {
      visualize(rrt, tree, env, vis);
    }
    return std::make_pair(solved, dt.toSec());
  };

  auto xg = goal.randomGoal();

  bool ds_param;
  double ds_prob;
  std::string env_param;
  Environment robo_env = Static;
  bool direct_sampling_en = false;
  double direct_sampling_prob = 0.5;
  if(ros::param::get("environment", env_param))
    robo_env = (env_param == std::string("dynamic") ? Dynamic : Static);
  if(ros::param::get("direct_sampling", ds_param))
    direct_sampling_en = ds_param;
  if(ros::param::get("direct_sampling_prob", ds_prob))
    direct_sampling_prob = ds_prob;

  if(direct_sampling_en) {
    sampler.set_direct_sample(true, direct_sampling_prob);
    sampler_dyn.set_direct_sample(true, direct_sampling_prob);
    sampler.target = xg;
    sampler_dyn.target = xg;
  }

  // receive ros messsage in separate threads
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // wait for topic to be published
  ros::Rate rate(3);
  while(ros::ok() && !subs.numPublishers()) {
    ROS_INFO("waiting for topic");
    rate.sleep();
  }

  auto max_iter = 100;
  auto solved = false;
  auto time = 0.0;

  while(ros::ok()) {
    switch(robo_env) {
    case Static :
    {
      auto sol = solve_rrt(rrt, subs, env, tree, &xg, max_iter);
      solved = std::get<0>(sol);
      time = std::get<1>(sol);
      break;
    }
    case Dynamic :
    {
      auto sol = solve_rrt(rrt_dyn, subs, env_dyn, tree, &xg, max_iter);
      solved = std::get<0>(sol);
      time = std::get<1>(sol);
      break;
    }
    }
    ROS_INFO("solution... xg(%f,%f,%f,%f) %s in %f s", xg(0), xg(1), xg(2), xg(3),
             (solved ? "found" : "not found"), time);
  }

  return 0;
}
