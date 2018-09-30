#include <fenv.h>
#include <boost/filesystem.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include "integrator2drrt.hpp"

int main(int argc, char** argv)
{
  // enable nan exception
  // feenableexcept(FE_DIVBYZERO);

  ros::init(argc, argv, "rrtstar_integrator2d");
  ros::NodeHandle node;

  Models::init_integrator2d();
  RRTVisual vis(node);

  auto &rrt = Kinodynamic::rrtstar_int2d;
  auto &tree = Kinodynamic::tree_int2d;

  auto xs = Kinodynamic::state_t();
  auto xg = Kinodynamic::state_t();

  std::array<double,4> s = {3.0, 3.0, 1.0, -1.0};
  std::array<double,4> g = {1.0, 1.0, 0.0, 0.0};
  for(size_t i=0; i<4; i++) {
    xs[i] = s[i];
    xg[i] = g[i];
  }
  // test trajectory
  /*
  auto trajectory = Kinodynamic::Trajectory<double,Kinodynamic::state_t,10>
      (Models::integrator2d_trj_solver.solve(
         (Models::Integrator2DSS::StateType)xs,
         (Models::Integrator2DSS::StateType)xg)
       );
  auto path = trajectory.path();
  std::stringstream ss;
  ss << "test trajectory : \n";
  ss << "start : \n" << xs << "\ngoal : \n" << xg << std::endl;
  for(const auto& p : path)
    ss << p << std::endl;
  ROS_INFO("%s", ss.str().c_str());
  */

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

  std::string dir("1");
  std::string motion_dir;
  std::string info_file("info.txt");
  std::string tree_file, parent_file, trajectory_file, environment_file, collision_file;
  std::stringstream info_data;

  enum DATA_STATUS {
    FIRST_DATA,
    UNFINISHED,
    NEW_DATA
  } status = FIRST_DATA;

  if(!node.getParam("/motion_dir", motion_dir))
    motion_dir = home+"/data/motion/";
  std::fstream stream(motion_dir+info_file, std::ios::in | std::ios::out);
  if(stream.peek() != std::ifstream::traits_type::eof()) {
    std::string str;
    while(std::getline(stream, str)) {
      std::string sstr;
      std::istringstream iss(str);
      std::vector<std::string> vstr;
      while(std::getline(iss, sstr, ' '))
        vstr.push_back(sstr);
      if(std::stoi(vstr.back()) < target_tree_size) {
        dir = vstr.front();
        status = UNFINISHED;
      }
      else if(status != UNFINISHED){
        info_data << str << "\n";
        dir = std::to_string(std::stoi(dir)+1);
        status = NEW_DATA;
      }
      else info_data << str << "\n";
    }
  }
  stream.close();

  boost::filesystem::path path(motion_dir+dir);
  if(!(boost::filesystem::exists(path))){
    ROS_INFO("%s doesn't exist", (motion_dir+dir).c_str());
    if (boost::filesystem::create_directory(path))
      ROS_INFO("%s created", (motion_dir+dir).c_str());
  }

  std::array<std::tuple<double,double>,9> obs;
  for(auto& o : obs)
    o = std::make_tuple(13.0,9.0);
  Kinodynamic::checker.env = Robosoccer<double,9>(obs);

  tree_file = motion_dir + dir + "/tree.txt";
  parent_file = motion_dir + dir + "/parent.txt";
  trajectory_file = motion_dir + dir + "/trajectory.txt";
  environment_file = motion_dir + dir + "/environment.txt";
  collision_file = motion_dir + dir + "/collisions.txt";

  if(status == UNFINISHED) {
    tree.from_text(tree_file, parent_file, trajectory_file);
    Kinodynamic::checker.from_text(environment_file);
  }
  else {
#ifndef NO_OBS
    Kinodynamic::checker.setRandomObstacles();
#endif
    xs = Kinodynamic::sampler();
    rrt.setStart(xs);
  }

  rrt.setIteration(tree.tree.size());
  bool first_draw = true;
  while(ros::ok()) {
    ROS_INFO("growing tree..");
    rrt.grow();
    auto tree_size = tree.tree.size();
    ROS_INFO("tree size : %d;", tree_size);

    // visualize the tree
    if(((tree_size % 100) == 0)
       || (first_draw)) {
      first_draw = false;
      ROS_INFO("adding visual..");
      auto r = Kinodynamic::checker.env.collision_radius;
      for(const auto &o : Kinodynamic::checker.env.obs)
        vis.add_circles(std::get<0>(o),std::get<1>(o),r);
      vis.add_trajectories(tree.tree.cloud.states, tree.trajectories, tree.parent, 2, tree_size);
      ROS_INFO("publish visual..");
      vis.delete_all();
      vis.publish();
      vis.clear();
    }

    // the tree size reached target, save and re-compute with new start
    if(tree_size >= target_tree_size) {
      // save current tree & reset
      std::ofstream info_stream(motion_dir+info_file);
      info_data << dir << " " << tree_size << "\n";
      info_stream << info_data.str();
      info_stream.close();
      tree.dump_text(tree_file,parent_file,trajectory_file);
      Kinodynamic::checker.dump_text(environment_file, collision_file);
      tree.reset();

      // create new directory
      dir = std::to_string(std::stoi(dir)+1);
      tree_file = motion_dir + dir + "/tree.txt";
      parent_file = motion_dir + dir + "/parent.txt";
      trajectory_file = motion_dir + dir + "/trajectory.txt";
      environment_file = motion_dir + dir + "/environment.txt";
      collision_file = motion_dir + dir + "/collisions.txt";
      boost::filesystem::path path(motion_dir+dir);
      if(!(boost::filesystem::exists(path))){
        ROS_INFO("%s doesn't exist", (motion_dir+dir).c_str());
        if (boost::filesystem::create_directory(path))
          ROS_INFO("%s created", (motion_dir+dir).c_str());
      }

      // new sample
      Kinodynamic::checker.setRandomObstacles();
      xs = Kinodynamic::sampler();
      rrt.setStart(xs);
      rrt.setIteration(0);
    }
    // rate.sleep();
    duration.sleep();
  }

  std::ofstream info_stream(motion_dir+info_file);
  info_data << dir << " " << tree.tree.size() << "\n";
  info_stream << info_data.str();
  info_stream.close();
  tree.dump_text(tree_file,parent_file,trajectory_file);
  Kinodynamic::checker.dump_text(environment_file, collision_file);

  // test trajectory
  /*
  ss.str() = std::string("");
  ss << "tree.trajectories.at(0) :\n";
  auto trj = tree.trajectories.back().path();
  for(const auto &p : trj)
    ss << p << std::endl;
  std::cout << ss.str();
  */
  return 0;
}
