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
      auto h = heading;
      heading = msg->robotinfo.at(i).heading.theta;
      heading_rate = heading - h;
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

  auto dt = time - last_recv;
  auto nf = (1.0/dt.toSec());
  f = (isnan(nf) || isinf(nf) ? f : nf);
  auto osize = msg->obstacleinfo.pos.size();
  std::vector<size_t> ids;
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
      // make it smooth
      constexpr auto alpha = 0.97;
      get<0>(near) = alpha * get<0>(near) + (1.0 - alpha) * pos.x;
      get<1>(near) = alpha * get<1>(near) + (1.0 - alpha) * pos.y;
      get<2>(near) = (get<0>(near) - get<0>(p0)) * f;
      get<3>(near) = (get<1>(near) - get<1>(p0)) * f;
      ids.push_back(id);
    }
  }
  last_recv = time;
  //  ROS_INFO("f : %f", f);
  //  for(auto o : obs)
  //    ROS_INFO("(%f,%f,%f,%f)", o(0), o(1), o(2), o(3));
}
