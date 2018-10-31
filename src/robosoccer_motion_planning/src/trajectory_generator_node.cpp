#include <ros/ros.h>
#include <type_traits>
#include "trajectory1d.h"
#include <rrtstar/TrajectoryService.h>

bool trajectory_server(rrtstar::TrajectoryServiceRequest &req, rrtstar::TrajectoryServiceResponse &res)
{
  if(req.model==std::string("angular_trajectory")) {
    Trajectory1D::AngleController angular_trajectory;
    if(req.params.size()>1) {
      ROS_INFO("setting trajectory limit");
      angular_trajectory.setLimit(req.params.at(0),req.params.at(1));
    }
    double t;
    std::decay_t<decltype(angular_trajectory.angleControl({0.0,0.0},0.0,t))> ctrl;
    std::decay_t<decltype(angular_trajectory.getOffset())> offset;
    // std::decay_t<decltype(angular_trajectory.getAngleTrajectory(ctrl,offset,0.0,t,25))> trajectory;
    if(req.xi.n>1)
    {
      ROS_INFO("solving angular control");
      // note that this fn only return the control sequence not the trajectory itself
      ctrl = angular_trajectory.angleControl({req.xi.state.at(0),req.xi.state.at(1)},req.xf.state.at(0),t);
      // since the controller normalize the initial pos to zero, we got an offset value
      offset = angular_trajectory.getOffset();
      ROS_INFO("angular control solved");
    }
    // the first is state, second is time;
    ROS_INFO("solving angular trajectory");
    std::vector<double> u;
    auto trajectory = angular_trajectory.getAngleTrajectory(ctrl,offset,0.0,t,25,&u);
    ROS_INFO("angular trajectory solved");
    auto s_it = trajectory.first.begin(); auto t_it = trajectory.second.begin();
    auto u_it = u.begin();
    for(; s_it!=trajectory.first.end() && t_it !=trajectory.second.end(); s_it++, t_it++, u_it+=(u_it==u.end()?0:1)) {
      res.model = std::string("angular_trajectory");
      res.time = t;
      std::decay_t<decltype(res.trajectory.at(0))> state;
      std::decay_t<decltype(res.inputs.at(0))> input;
      state.n = 2;
      input.n = 1;
      if(u_it!=u.end()) {
        input.state.push_back(*u_it);
        input.t.data = ros::Time(*t_it);
      }
      state.state.push_back((*s_it).w);
      state.state.push_back((*s_it).dw);
      state.t.data = ros::Time(*t_it);
      res.trajectory.push_back(state);
      res.inputs.push_back(input);
    }
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_generator_node");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("compute_trajectory", trajectory_server);
  ROS_INFO("Ready to compute trajectory.");
  ros::spin();

  ROS_INFO("Hello world!");
}
