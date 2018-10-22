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

  // read parameter
  bool ds_param;
  double ds_prob;
  int ts;
  std::string env_param;
  size_t target_size = 100;
  Environment robo_env = Static;
  bool direct_sampling_en = false;
  double direct_sampling_prob = 0.5;
  if(ros::param::get("environment", env_param))
    robo_env = (env_param == std::string("dynamic") ? Dynamic : Static);
  if(ros::param::get("direct_sampling", ds_param))
    direct_sampling_en = ds_param;
  if(ros::param::get("direct_sampling_prob", ds_prob))
    direct_sampling_prob = ds_prob;
  if(ros::param::get("target_size", ts))
    target_size = ts;

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
  auto vis_t0 = ros::Time::now();
  auto vis_t1 = ros::Time::now();
  auto solve_rrt = [set_start, set_obstacles, visualize, &vis, &vis_t0, &vis_t1]
      (auto &rrt, auto &subs, auto &env, auto &tree, auto *xg, auto &sampler, size_t iteration)
  {
    set_start(rrt, subs);
    set_obstacles(env, subs);
    auto solved = false;
    auto t0 = ros::Time::now();
    ROS_INFO("growing tree");
    auto ts = tree.tree.size();
    for(size_t i=0; i<iteration; i++) {
      solved = rrt.grow(xg);
      auto s = sampler.last_sample();
      ROS_INFO("sample : %f, %f, %f, %f %s", s(0), s(1), s(2), s(3), (tree.tree.size() > ts ? "ok" : "failed"));
      ts = tree.tree.size();
    }
    auto t1 = ros::Time::now();
    auto dt = t1 - t0;
    vis_t1 = t0;
    auto vis_dt = vis_t1 - vis_t0;
    if((tree.tree.size() > 0) && (vis_dt.toSec() > 0.066)) {
      visualize(rrt, tree, env, vis);
      vis_t0 = vis_t1;
    }
    return std::make_pair(solved, dt.toSec());
  };

  // receive ros messsage in separate threads
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // wait for topic to be published
  ros::Rate rate(3);
  while(ros::ok() && !subs.numPublishers()) {
    ROS_INFO("waiting for topic");
    rate.sleep();
  }

  auto solved = false;
  auto time = 0.0;

  while(subs.getObstacles().size() < 9) {
    ROS_INFO("waiting for models to be spawned");
    rate.sleep();
  }

  set_obstacles(env, subs);
  set_obstacles(env_dyn, subs);

  auto xg = goal.randomGoal();

  if(direct_sampling_en) {
    sampler.set_direct_sample(true, direct_sampling_prob);
    sampler_dyn.set_direct_sample(true, direct_sampling_prob);
    sampler.target = xg;
    sampler_dyn.target = xg;
    ROS_INFO("direct sampling enabled with prob : %f", sampler_dyn.direct_sampler->p[0]);
  }

  while(ros::ok()) {
    switch(robo_env) {
    case Static :
    {
      auto sol = solve_rrt(rrt, subs, env, tree, &xg, sampler, target_size);
      solved = std::get<0>(sol);
      time = std::get<1>(sol);
      break;
    }
    case Dynamic :
    {
      auto sol = solve_rrt(rrt_dyn, subs, env_dyn, tree, &xg, sampler_dyn, target_size);
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
