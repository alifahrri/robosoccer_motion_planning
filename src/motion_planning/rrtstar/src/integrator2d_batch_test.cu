#include <boost/filesystem.hpp>
#include "integrator2drrt.cuhpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrtstar_integrator2d_test");
  ros::NodeHandle node;

  Models::init_integrator2d();
  RRTVisual vis(node, "test");

  auto &rrt = Kinodynamic::rrtstar_batch_int2d;
  auto &tree = Kinodynamic::tree_int2d;
  auto &env = Kinodynamic::robosoccer_env_cuda;

  auto xs = Kinodynamic::state_t();
  auto xg = Kinodynamic::state_t();

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

  while(ros::ok()) {
    Kinodynamic::batch_checker.setRandomObstacles();
    xs = Kinodynamic::sampler();
    rrt.setStart(xs);
    rrt.setIteration(0);
    bool solved = false;
    auto t0 = ros::Time::now();
    auto vis_t0 = ros::Time::now();
    auto xg = Kinodynamic::goal.randomGoal();
    while((!solved) && ros::ok()) {
      ROS_INFO("growing tree..");
      rrt.grow();
      auto tree_size = tree.tree.size();
      ROS_INFO("tree size : %d;", tree_size);
      if((tree_size % 5) == 0) {
        ROS_INFO("inserting goal..");
        ROS_INFO("finding solution... xg(%f,%f,%f,%f)",xg(0),xg(1),xg(2),xg(3));
        solved = rrt.insertGoal(xg);
      }
      auto vis_t1 = ros::Time::now();
      auto dt = vis_t1-vis_t0;
      ROS_INFO("dt : %f", dt.toSec());

      if(dt.toSec() > 0.5) {
        ROS_INFO("adding visual..");
        auto r = env.collision_radius;
        for(const auto &o : env.obs)
          vis.add_obstacles(o.x,o.y,r);
        vis.set_trajectories(tree.tree.cloud.states, tree.trajectories, tree.parent, 2, tree.tree.size());
        ROS_INFO("publish visual..");
        vis.delete_all();
        vis.publish();
        vis.clear();
        vis_t0 = vis_t1;
      }
    }
    auto t1 = ros::Time::now();

    ROS_INFO("solution found in %f", (t1-t0).toSec());
    // visualize the tree
    ROS_INFO("adding visual..");
    auto r = env.collision_radius;
    for(const auto &o : env.obs)
      vis.add_obstacles(o.x,o.y,r);
    vis.set_trajectories(tree.tree.cloud.states, tree.trajectories, tree.parent, 2, tree.tree.size());
    ROS_INFO("publish visual..");
    vis.delete_all();
    vis.publish();
    vis.clear();
    // rate.sleep();
    duration.sleep();
  }

  return 0;
}
