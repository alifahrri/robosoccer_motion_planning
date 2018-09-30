#include <ros/ros.h>
#include <rrtstar.hpp>
#include <integrator2drrt.hpp>
#include "robotsubscriber.h"

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
  auto &tree = Kinodynamic::tree_int2d;

  auto set_start = [&rrt, &subs] () {
    auto s = subs.getState();
    Kinodynamic::TreeInt2D::State xs;
    xs(0) = s(0); xs(1) = s(1);
    xs(2) = s(2); xs(3) = s(3);
    ROS_INFO("xs(%f,%f,%f,%f)",xs(0),xs(1),xs(2),xs(3));
    rrt.setStart(xs);
    rrt.setIteration(0);
  };

  auto set_obstacles = [&rrt, &subs, &env] () {
    env.setObstacles(subs.getObstacles());
  };

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // wait for topic to be published
  ros::Rate rate(3);
  while(ros::ok() && !subs.numPublishers()) {
    ROS_INFO("waiting for topic");
    rate.sleep();
  }

  auto xg = Kinodynamic::goal.randomGoal();

  while(ros::ok()) {
    set_start();
    set_obstacles();
    auto solved = false;
    auto max_iter = 100;
    auto t0 = ros::Time::now();
    for(size_t i=0; i<max_iter; i++) {
      solved = rrt.grow(&xg);
      // solved = rrt.insertGoal(xg);
      // if(solved) break;
    }
    auto t1 = ros::Time::now();
    auto dt = t1 - t0;
    ROS_INFO("solution... xg(%f,%f,%f,%f) %s in %f s",xg(0),xg(1),xg(2),xg(3),(solved ? "found" : "not found"), dt.toSec());
    if(tree.tree.size() > 0) {
      vis.clear();
      ROS_INFO("adding visual..");
      auto r = Kinodynamic::checker.env.collision_radius;
      for(const auto &o : Kinodynamic::checker.env.obs)
        vis.add_circles(std::get<0>(o),std::get<1>(o),r);
      vis.add_trajectories(tree.tree.cloud.states, tree.trajectories, tree.parent, 2, tree.tree.size());
      if(solved) {
        auto goal_trj = tree.get_trajectory(rrt.goalIndex());
        for(const auto &t : goal_trj)
          vis.add_trajectory(t.path(),2);
      }
      ROS_INFO("publish visual..");
      vis.publish();
    }
  }

  return 0;
}
