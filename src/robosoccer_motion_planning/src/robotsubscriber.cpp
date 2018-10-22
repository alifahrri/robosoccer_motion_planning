#include "robotsubscriber.h"

using namespace std;

RobotSubscriber::RobotSubscriber(ros::NodeHandle &node, size_t robot_id, TEAM team)
  : id(robot_id)
{
  string topic_name;
  switch (team) {
  case NUBOT:
    topic_name = string("/nubot") + to_string(id) + "/omnivision/OmniVisionInfo";
    break;
  case RIVAL:
    topic_name = string("/rival") + to_string(id) + "/omnivision/OmniVisionInfo";
    break;
  default:
    break;
  }
  sub = node.subscribe<nubot_common::OminiVisionInfo>(topic_name, 3, &RobotSubscriber::callback, this);
}

void RobotSubscriber::callback(const nubot_common::OminiVisionInfo::ConstPtr &msg)
{
  auto time = msg->header.stamp;
  for(size_t i=0; i<msg->robotinfo.size(); i++) {
    auto agent_id = msg->robotinfo.at(i).AgentID;
    if(agent_id == id) {
      auto px = msg->robotinfo.at(i).pos.x/100.0;
      auto py = msg->robotinfo.at(i).pos.y/100.0;
      auto vx = msg->robotinfo.at(i).vtrans.x/100.0;
      auto vy = msg->robotinfo.at(i).vtrans.y/100.0;
      get<0>(state) = px; get<1>(state) = py;
      get<2>(state) = vx; get<3>(state) = vy;
      heading = msg->robotinfo.at(i).heading.theta;
      // ROS_INFO("robot info(%f,%f,%f,%f)", px, py, vx, vy);
      break;
    }
  }

  // select nearest obs, returning index
  auto select_obs = [](auto &pos, auto &obs, auto &exclude) {
    auto id = 0;
    auto dis = 1e3;
    for(size_t i=0; i<obs.size(); i++) {
      auto o = obs.at(i);
      auto d = hypot(pos.x-get<0>(o), pos.y-get<1>(o));
      if(d < dis) {
        for(auto idx : exclude) {
          if(idx != i) {
            id = i;
            dis = d;
          }
        }
      }
    }
    return id;
  };

  auto least_square_fit = [](auto &xdata, auto &ydata, auto get_x, auto get_y) {
    decltype(get_x(xdata.front())) x_sum(0);
    decltype(get_y(ydata.front())) y_sum(0);
    decltype(get_x(xdata.front())) xy_sum(0);
    decltype(get_x(xdata.front())) xsq_sum(0);
    decltype(get_y(ydata.front())) ysq_sum(0);
    auto n = xdata.size();
    for(size_t i=0; i<xdata.size(); i++)
    {
      auto x = get_x(xdata[i]);
      auto y = get_y(ydata[i]);
      x_sum = x_sum + x;
      y_sum = y_sum + y;
      xsq_sum = xsq_sum + x*x;
      ysq_sum = ysq_sum + y*y;
      xy_sum = xy_sum + x*y;
    }
    // f(a, b) = a + bx
    auto c = 1.0 / (n * xsq_sum -  x_sum * x_sum);
    auto a = c * (y_sum * xsq_sum - x_sum * xy_sum);
    auto b = c * (n * xy_sum - x_sum * y_sum);
    return make_pair(a, b);
  };
  // obs.clear();

  auto dt = time - last_recv;
  auto nf = (1.0/dt.toSec());
  f = (isnan(nf) || isinf(nf) ? f : nf);
  auto osize = msg->obstacleinfo.pos.size();
  std::vector<size_t> ids;
//  if(obs.size() >= n_obstacles) {
//    if(time_history.size() < n_obstacles) {
//      time_history.resize(n_obstacles);
//      obs_history.resize(n_obstacles);
//    }
//    for(size_t i=0; i<obs.size(); i++) {
//      obs_history.at(i).push_back(obs.at(i));
//      time_history.at(i).push_back(time);
//    }
//  }
  for(size_t i=0; i<osize; i++) {
    State o;
    auto pos = msg->obstacleinfo.pos.at(i);
    pos.x /= 100.0;
    pos.y /= 100.0;
    if(obs.size() <= i) {
      get<0>(o) = pos.x;
      get<1>(o) = pos.y;
      get<2>(o) = .0;
      get<3>(o) = .0;
      obs.push_back(o);
    }
    else {
      auto id = select_obs(pos, obs, ids);
      auto& near = obs.at(id);
      std::decay_t<decltype(obs.at(id))> p0 = obs.at(id);
      get<0>(near) = pos.x;
      get<1>(near) = pos.y;
//      if(obs_history.at(id).size() >= 10) {
//        auto x_fn = least_square_fit(time_history.at(id), obs_history.at(id),
//        [&](auto t){ return (t - time_history.at(id).front()).toSec(); },
//        [&](auto f){ return f(0); });
//        auto y_fn = least_square_fit(time_history.at(id), obs_history.at(id),
//        [&](auto t){ return (t - time_history.at(id).front()).toSec(); },
//        [&](auto f){ return f(1); });
//        get<2>(near) = x_fn.second;
//        get<3>(near) = y_fn.second;
//        time_history.at(id).erase(time_history.at(id).begin());
//        obs_history.at(id).erase(obs_history.at(id).begin());
//      }
      get<2>(near) = (get<0>(near) - get<0>(p0)) * f;
      get<3>(near) = (get<1>(near) - get<1>(p0)) * f;
      ids.push_back(id);
    }
  }
  last_recv = time;
  ROS_INFO("f : %f", f);
  for(auto o : obs)
    ROS_INFO("(%f,%f,%f,%f)", o(0), o(1), o(2), o(3));
}
