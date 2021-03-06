#include <ros/ros.h>
#include <type_traits>
#include <rrtstar.hpp>
#include "trajectory1d.h"
#include "goalsubscriber.h"
#include "robotsubscriber.h"
#include "integrator2drrt.hpp"
#include "robosoccer_visual.hpp"
#include "trajectorypublisher.h"

enum Environment {
  Static,
  Dynamic
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robosoccer_motion_planning");
  ros::NodeHandle node;

  int id_param;
  std::string g_param;
  size_t agent_id = 1;
  std::string goal_topic("move_base_simple/goal");
  // read agent id parameter to decide which agent should be controlled
  // read topic name from rosparam
  if(ros::param::get("agent_id", id_param))
    agent_id = id_param;
  if(ros::param::get("goal_topic", g_param))
    goal_topic = g_param;

  // ceate visualisation publisher
  RRTVisual vis(node);

  // create subscriber for robot and obstacle pos
  // create publisher for computed trajectory
  // create goal subscriber
  auto subs = RobotSubscriber(node, agent_id, RobotSubscriber::NUBOT);
  auto pubs = TrajectoryPublisher(node, "robosoccer_trajectory_pos", "robosoccer_trajectory_vel", subs.getName());
  auto goal_subs = GoalSubscriber(node, goal_topic);

  // actually this is no longer needed
  Models::init_integrator2d();

  // for static obstacle :
  auto &rrt = Kinodynamic::Wrapper::get_rrtstar_int2d();
  auto &env = Kinodynamic::Wrapper::get_robosoccer_env();
  auto &goal = Kinodynamic::Wrapper::get_goal();
  auto &sampler = Kinodynamic::Wrapper::get_sampler();

  // for dynamic obstacle :
  auto &rrt_dyn = Kinodynamic::Wrapper::get_rrtstar_int2d_timespace_obs();
  auto &env_dyn = Kinodynamic::Wrapper::get_dynamic_soccer_env();
  auto &goal_dyn = Kinodynamic::Wrapper::get_goal_dynamic_env();
  auto &sampler_dyn = Kinodynamic::Wrapper::get_sampler_dynamic_env();

  // lqr solver
  auto &solver = Kinodynamic::Wrapper::get_trajectory_solver();

  // current implementation share the tree between static & dyn obst.
  auto &tree = Kinodynamic::Wrapper::get_tree_int2d();

  int ts; bool ds_param;
  std::string env_param;
  size_t target_size = 100;
  double vwmax = 0.5, awmax = 0.5;
  Environment robo_env = Static;
  bool direct_sampling_en = false;
  double vis_rate = 7.0;
  double ds_prob, vr_param, cr_param;
  double direct_sampling_prob = 0.5;
  // read parameter
  if(ros::param::get("environment", env_param))
    robo_env = (env_param == std::string("dynamic") ? Dynamic : Static);
  if(ros::param::get("direct_sampling", ds_param))
    direct_sampling_en = ds_param;
  if(ros::param::get("direct_sampling_prob", ds_prob))
    direct_sampling_prob = ds_prob;
  if(ros::param::get("target_size", ts))
    target_size = ts;
  if(ros::param::get("rrtvis_rate", vr_param))
    vis_rate = vr_param;
  if(ros::param::get("collision_radius", cr_param)) {
    env.collision_radius = cr_param;
    env_dyn.collision_radius = cr_param;
  }
  if(ros::param::has("r")) {
    double r;
    ros::param::get("r",r);
    ROS_WARN("setting input weight for trajectory solver");
    auto &solver = Kinodynamic::Wrapper::get_trajectory_solver();
    solver.set_weight(r);
  }
  if(ros::param::has("max_angular_velocity"))
    ros::param::get("max_angular_velocity", vwmax);
  if(ros::param::has("max_angular_acceleration"))
    ros::param::get("max_angular_acceleration", awmax);

  // for trajectory tracking testing which means trajectory only published once
  bool publish_trajectory = true;
  bool trajectory_tracking_test = false;
  if(ros::param::has("trajectory_tracking_test")) {
    ros::param::get("trajectory_tracking_test",trajectory_tracking_test);
    ROS_WARN("trajectory tracking test : %s", (trajectory_tracking_test ? "enabled" : "disabled"));
  }

  // create a trajectory generator for angle
  Trajectory1D::AngleController angular_trajectory;
  angular_trajectory.setLimit(vwmax, awmax);

  // generic lambdas :
  // set starting state for rrt given the subscriber
  std::decay_t<decltype(subs.getHeading())> ws;
  std::decay_t<decltype(subs.getState())> *next_start = nullptr;
  std::decay_t<decltype(subs.getHeading())> *next_wstart = nullptr;
  // we capture ws here to make sure that pose that used for rrt and
  // angle trajectory generator is received at the same time point
  auto set_start = [&ws, &pubs, &next_start, &next_wstart](auto &rrt, auto &subs) {
    auto s = subs.getState();
    ws = subs.getHeading();
    if(next_start){
      auto& ns = *next_start;
      auto d = std::hypot(s(0)-ns(0), s(1)-ns(1));
      if(d < 1.0f){
        s = *next_start;
        if(next_wstart)
          ws = *next_wstart;
      }
    }
    pubs.set_initial(s, ws.first);
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
    call_vis(rrt, tree, env, vis);
    // clear all before re-drawing
    vis.delete_all();
    vis.publish();
    vis.clear();
  };
  auto set_goal = [&goal_subs, &publish_trajectory, &trajectory_tracking_test, &next_start, &next_wstart](auto &goal, auto &yaw)
  {
    if(goal_subs.goalChanged()){
      delete next_start;
      delete next_wstart;
      next_start = nullptr;
      next_wstart = nullptr;
      publish_trajectory = true;
    }
    else if(trajectory_tracking_test)
      publish_trajectory = false;
    yaw = goal_subs.getYaw();
    goal_subs.getGoal(goal);
  };
  auto vis_t0 = ros::Time::now();
  auto vis_t1 = ros::Time::now();
  // create helper lambda to run the rrt
  auto solve_rrt = [set_start, set_obstacles, visualize, &vis, &vis_t0, &vis_t1, &vis_rate, &goal_subs, &direct_sampling_en]
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
      // make sure at least we sample to the goal at the end and at the middle
      // if direct sampling is enabled
      if(direct_sampling_en && ((i == (iteration-2)) || (i == (iteration/2))))
        sampler.set_next_sample(*xg);
    }
    auto t1 = ros::Time::now();
    auto dt = t1 - t0;
    vis_t1 = t0;
    auto vis_dt = vis_t1 - vis_t0;
    // visualize tree on certain rate
    if((tree.tree.size() > 0) && (vis_dt.toSec() > (1./vis_rate))) {
      visualize(rrt, tree, env, vis);
      vis_t0 = vis_t1;
    }
    return std::make_pair(solved, dt.toSec());
  };

  // receive ros messsage in separate threads
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // wait for topic to be published
  ros::Rate rate(3);
  while(ros::ok() && !subs.numPublishers()) {
    ROS_INFO("waiting for topic");
    rate.sleep();
  }

  auto solved = false;
  auto time = 0.0;

  // self-explanatory
  while(subs.getObstacles().size() < 9) {
    ROS_INFO("waiting for models to be spawned : %d", subs.getObstacles().size());
    rate.sleep();
  }

  set_obstacles(env, subs);
  set_obstacles(env_dyn, subs);

  auto xg = goal.randomGoal();
  goal_subs.setGoal(xg);

  if(direct_sampling_en) {
    sampler.set_direct_sample(true, direct_sampling_prob);
    sampler_dyn.set_direct_sample(true, direct_sampling_prob);
    sampler.target = xg;
    sampler_dyn.target = xg;
    ROS_INFO("direct sampling enabled with prob : %f", sampler_dyn.direct_sampler->p[0]);
  }

  auto get_trajectory = [&tree, &solver](auto idx)
  {
    // basic trajectory (array)
    // return tree.get_trajectory(rrt.goalIndex());
    // high quality trajectory
    auto states = tree.states(idx);
    auto dt = 0.1;
    Kinodynamic::VTrajectory ret;
    // re-solve kinodynamic trajectory with finer grain
    for(size_t i=1; i<states.size(); i++) {
      auto tc = solver.solver.cost(states[i-1],states[i]);
      auto t = std::get<0>(tc);
      auto c = std::get<1>(tc);
      auto trajectory = solver.solver.solve(states[i-1],states[i],dt);
      ret.insert(trajectory);
    }
    return ret;
  };

  while(ros::ok()) {
    // declare trajectory, the type is automatically deduced
    // decltype(tree.get_trajectory(0)) trajectory;
    // use high quality trajectory :
    decltype(get_trajectory(0)) trajectory;
    // save current time to be used for trajectory publisher
    auto rrt_t0 = ros::Time::now();
    // set goal based on subscribed topic
    auto yaw = 0.0;
    set_goal(xg, yaw);
    // we should update target for sampler too!
    if(direct_sampling_en) {
      sampler.target = xg;
      sampler_dyn.target = xg;
    }
    switch(robo_env) {
    case Static :
    {
      auto sol = solve_rrt(rrt, subs, env, tree, &xg, sampler, target_size);
      solved = std::get<0>(sol);
      time = std::get<1>(sol);
      // if(solved) trajectory = tree.get_trajectory(rrt.goalIndex());
      // use high quality trajectory ;
      if(solved) trajectory = get_trajectory(rrt.goalIndex());
      break;
    }
    case Dynamic :
    {
      auto sol = solve_rrt(rrt_dyn, subs, env_dyn, tree, &xg, sampler_dyn, target_size);
      solved = std::get<0>(sol);
      time = std::get<1>(sol);
      // if(solved) trajectory = tree.get_trajectory(rrt_dyn.goalIndex());
      // use hight quality trajectory :
      if(solved) trajectory = get_trajectory(rrt_dyn.goalIndex());
      break;
    }
    }
    ROS_WARN("solution... xg(%f,%f,%f,%f) %s in %f s", xg(0), xg(1), xg(2), xg(3),
             (solved ? "found" : "not found"), time);
    if(solved)
    {
      // concatenate the trajectory vector (of array of state, which is definition of edges) to vector of states
      // std::vector<std::decay_t<decltype(trajectory.at(0).at(0))>> trj;
      // for(auto t : trajectory) trj.insert((trj.size() ? trj.end() : trj.begin()), t.begin(), t.end());

      // for hq trajectory, we don't need to concatenate (already a vector of states):
      auto trj = trajectory;

      // some type deduction
      using State = std::decay_t<decltype(trj.at(0))>;
      using Array = std::array<decltype(std::declval<State>()(0)),2>;

      // solve angular trajectory
      auto wt = 0.0;
      ROS_INFO("solving angular trajectory : (%f,%f) -> (%f,0.0)", std::get<0>(ws), std::get<1>(ws), yaw);
      auto ctrl = angular_trajectory.angleControl({std::get<0>(ws), std::get<1>(ws)}, yaw, wt);
      // create vector to hold time
      // we need this to match time index of int2d and angular trj1d
      // std::vector<std::decay_t<decltype(trj.front()(4))>> time_vec;
      // for(auto t : trj) time_vec.push_back(t(4));
      // auto dt = time_vec.at(1)-time_vec.at(0);
      auto offset = angular_trajectory.getOffset();
      // auto wtrj = Trajectory1D::AngleController::getAngleTrajectory(ctrl,offset, 0.0, wt, (int)time_vec.size());
      auto wtrj = Trajectory1D::AngleController::getAngleTrajectory(ctrl,offset, 0.0, wt, (int)trj.size());

      // define getter funtion for vector of state time (px, py, vx, vy, t)
      auto get_vel = [](const State &s){ return Array{s(2),s(3)}; };
      auto get_pos = [](const State &s){ return Array{s(0),s(1)}; };
      auto get_time = [&rrt_t0](const State &s){
        decltype(rrt_t0 + ros::Duration(std::max(s(4),decltype(s(4))(0.0)))) ret;
        try {
          ret = rrt_t0 + ros::Duration(std::max(s(4),decltype(s(4))(0.0)));
        }
        catch(std::runtime_error& ex){
          ret = rrt_t0;
        }
        return ret;
      };
      auto angle_fn = [&wtrj](size_t i, double &w, double &dw)
      {
        auto s = wtrj.first.at(i);
        w = s.w; dw = s.dw;
      };

      // pubs.set(trj, get_time, get_pos, get_vel);
      // also publish angle :
      pubs.set_pose2d(trj, get_time, get_pos, get_vel, angle_fn);
      if(publish_trajectory) {
        using next_start_t = std::decay_t<decltype(*next_start)>;
        using next_wstart_t = std::decay_t<decltype(*next_wstart)>;
        pubs.publish();
        if(!next_start) next_start = new next_start_t;
        if(!next_wstart) next_wstart = new next_wstart_t;
        auto now = ros::Time::now();
        pubs.get(*next_start,*next_wstart,now);
        // allows trajectory tracking testing
        if(trajectory_tracking_test)
          publish_trajectory = false;
      }
    }
  }

  return 0;
}
