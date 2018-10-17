#include <fenv.h>
#include <boost/filesystem.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include "integrator2drrt.hpp"
#include "environment.hpp"

typedef std::tuple<
Kinodynamic::state_t,
Kinodynamic::state_t,
//std::array<std::tuple<double,double>,9>,
Obstacles<double,9>,
std::vector<Kinodynamic::Trajectory<>>> Solution;
typedef std::vector<Solution> Solutions;

Solutions load_solutions(const std::string &file, const std::string &info_solution) {

}

std::vector<std::string> get_directory_list(const std::string& info_file, const std::string &info_solution, size_t target_size) {
  std::vector<std::string> dir_list;
  std::ifstream stream(info_file);
  std::ifstream sol_stream(info_solution);
  std::string str, sol_str;
  while(std::getline(stream, str)) {
    std::getline(sol_stream, sol_str);
    std::istringstream ss(str);
    std::istringstream ss_sol(sol_str);
    std::string sstr, sol_sstr;
    std::getline(ss, sstr, ' ');
    std::getline(ss_sol, sol_sstr, ' ');
    auto dir = sol_sstr;
    std::getline(ss_sol, sol_sstr, ' ');
    auto size = sol_sstr.empty() ? 0 : std::stoi(sol_sstr);
    if((sstr != dir) || (size<target_size))
      dir_list.push_back(sstr);
  }
  stream.close();
  sol_stream.close();
  return dir_list;
}

void save_solutions(const Solutions &sol, const std::string &file, const std::string &info_solution, const std::vector<std::string> &dir_list, std::vector<size_t> sol_size, size_t goal_per_dir) {
  std::stringstream ss;
  std::string str;

  std::ifstream istream(file);
  if(istream.is_open())
    while(std::getline(istream, str))
      ss << str << "\n";
  istream.close();

  for(const auto &s : sol) {
    const auto &xs = std::get<0>(s);
    const auto &xg = std::get<1>(s);
    const auto &ob = std::get<2>(s);
    const auto &tr = std::get<3>(s);
    ss << "("; //conditioning variable
    ss << xs(0) << "," << xs(1) << "," << xs(2) << "," << xs(3) << ",";
    ss << xg(0) << "," << xg(1) << "," << xg(2) << "," << xg(3) << ",";
    for(size_t i=0; i<ob.size(); i++)
      ss << std::get<0>(ob[i]) << "," << std::get<1>(ob[i]) << (i<(ob.size()-1) ? "," : "");
    ss << ")" << " ";
    ss << "("; // samples
    for(size_t i=0; i<tr.size(); i++) {
      auto path = tr[i].path();
      for(size_t j=0; j<path.size(); j++) {
        const auto &x = path[j];
        ss << x(0) << "," << x(1) << "," << x(2) << "," << x(3) << ((i*tr.size()+j)<(tr.size()*path.size()-1) ? "," : "");
      }
    }
    ss << ")";
    ss << "\n";
  }
  std::ofstream stream(file);
  stream << ss.str();
  stream.close();

  ss.str(std::string());

  std::ifstream iinfo(info_solution);
  if(iinfo.is_open()) {
    std::string str;
    while(std::getline(iinfo, str)) {
      std::string sstr;
      std::istringstream istream(str);
      std::getline(istream, sstr, ' ');
      std::getline(istream, sstr, ' ');
      if(std::stoi(sstr) >= goal_per_dir)
        ss << str << "\n";
    }
  }
  iinfo.close();

  for(size_t i=0; i<dir_list.size(); i++)
    ss << dir_list[i] << " " << sol_size[i] << "\n";
  std::ofstream info(info_solution);
  info << ss.str();
  info.close();
}

int main(int argc, char** argv)
{
  // enable nan exception
  // feenableexcept(FE_DIVBYZERO);

  ros::init(argc, argv, "rrtstar_integrator2d_dataset");
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

  ros::Rate rate(30.0f);
  ros::Duration duration(0,1);

  std::string home;
  ros::get_environment_variable(home, "HOME");

  int target_solution_size = 22*11;
  double neighbor_radius_scale = 1.0;
  if(!node.getParam("/target_solution_size",target_solution_size))
    target_solution_size = 22*11;
  if(node.getParam("/neighbor_radius_scale",neighbor_radius_scale))
    Kinodynamic::radius.scale = neighbor_radius_scale;

  std::string dir("1");
  std::string motion_dir;
  std::string info_file("info.txt");
  std::string tree_file, parent_file, trajectory_file, environment_file, collision_file;
  std::stringstream info_data;
  if(!node.getParam("/motion_dir", motion_dir))
    motion_dir = home+"/data/motion/";
  tree_file = motion_dir + dir + "/tree.txt";
  parent_file = motion_dir + dir + "/parent.txt";
  trajectory_file = motion_dir + dir + "/trajectory.txt";
  environment_file = motion_dir + dir + "/environment.txt";
  collision_file = motion_dir + dir + "/collisions.txt";
  auto info_solution = motion_dir + "/solutions_info.txt";

  auto directory_list = get_directory_list(motion_dir+info_file, info_solution, target_solution_size);

  Solutions solutions;
  bool first_draw = true;
  int sol_size = 0;
  std::vector<size_t> computed_sol;
  std::vector<std::string> computed_dir;
  while(ros::ok()) {
    for(const auto &s : directory_list) {
      sol_size = 0;
      tree_file = motion_dir + s + "/tree.txt";
      parent_file = motion_dir + s + "/parent.txt";
      trajectory_file = motion_dir + s + "/trajectory.txt";
      environment_file = motion_dir + s + "/environment.txt";
      ROS_INFO("loading %s %s %s", tree_file.c_str(), parent_file.c_str(), trajectory_file.c_str());
      tree.from_text(tree_file, parent_file, trajectory_file);
      ROS_INFO("loading %s", environment_file.c_str());
      Kinodynamic::checker.from_text(environment_file);
      rrt.setIteration(tree.tree.size());
      while((sol_size <= 0 || (sol_size % target_solution_size)) && ros::ok()){
        auto xg = Kinodynamic::goal.randomGoal();
        ROS_INFO("finding solution... xg(%f,%f,%f,%f)",xg(0),xg(1),xg(2),xg(3));
        auto found = rrt.insertGoal(xg,0);
        ROS_INFO("tree size : %d", tree.tree.size());
        ROS_INFO("solution %s", (found ? "found" : "not found"));
        if(found) {
          Solution sol;
          auto goal_id = rrt.goalIndex();
          auto trajectory = tree.get_trajectory(goal_id);
          std::get<0>(sol) = tree.tree(0);
          std::get<1>(sol) = tree.tree(goal_id);
          std::get<2>(sol) = Kinodynamic::robosoccer_env.obs;
          std::get<3>(sol) = trajectory;
          solutions.push_back(sol);
          sol_size = sol_size+1;
        }
        duration.sleep();
      }
      computed_dir.push_back(s);
      computed_sol.push_back(sol_size);
      if(!ros::ok())
        break;
    }
  }

  save_solutions(solutions, motion_dir+"solutions.txt", info_solution, computed_dir, computed_sol, target_solution_size);

  return 0;
}
