#include "robotsubscriber.h"
#include "elements.hpp"

using namespace std;

RobotSubscriber::RobotSubscriber(ros::NodeHandle &node, size_t robot_id, TEAM team)
  : id(robot_id)
{
  string topic_name;
  string rival_topic;
  switch (team) {
  case NUBOT:
    topic_name = string("/nubot") + to_string(id) + "/omnivision/OmniVisionInfo";
    rival_topic = string("/rival") + to_string(id) + "/omnivision/OmniVisionInfo";
    tf_name = string("nubot") + to_string(id) + "_tf";
    break;
  case RIVAL:
    topic_name = string("/rival") + to_string(id) + "/omnivision/OmniVisionInfo";
    rival_topic = string("/nubot") + to_string(id) + "/omnivision/OmniVisionInfo";
    tf_name = string("rival") + to_string(id) + "_tf";
    break;
  default:
    break;
  }
  sub = node.subscribe<nubot_common::OminiVisionInfo>(topic_name, 3, &RobotSubscriber::callback, this);
  rsub = node.subscribe<nubot_common::OminiVisionInfo>(rival_topic, 3, &RobotSubscriber::rival_callback, this);
}

void RobotSubscriber::rival_callback(const nubot_common::OminiVisionInfo::ConstPtr &msg)
{
  robs.clear();
  for(size_t i=0; i<msg->robotinfo.size(); i++) {
    auto robotinfo = msg->robotinfo.at(i);
    decay_t<decltype(robs.front())> s;
    // for rival, robot pos is flipped; nubot's rule
    x(s) = -x(robotinfo.pos)*1e-2;
    y(s) = -y(robotinfo.pos)*1e-2;
    vx(s) = -x(robotinfo.vtrans)*1e-2;
    vy(s) = -y(robotinfo.vtrans)*1e-2;
    robs.push_back(s);
  }
}

void RobotSubscriber::callback(const nubot_common::OminiVisionInfo::ConstPtr &msg)
{
  obs.clear();
  auto time = msg->header.stamp;
  for(size_t i=0; i<msg->robotinfo.size(); i++) {
    auto agent_id = msg->robotinfo.at(i).AgentID;
    auto robotinfo = msg->robotinfo.at(i);
    auto pos_x = x(robotinfo.pos)/100.;
    auto pos_y = y(robotinfo.pos)/100.;
    auto vel_x = x(robotinfo.vtrans)/100.;
    auto vel_y = y(robotinfo.vtrans)/100.;
    if(agent_id == id) {
      //      auto px = msg->robotinfo.at(i).pos.x/100.0;
      //      auto py = msg->robotinfo.at(i).pos.y/100.0;
      //      auto vx = msg->robotinfo.at(i).vtrans.x/100.0;
      //      auto vy = msg->robotinfo.at(i).vtrans.y/100.0;
      //      get<0>(state) = px; get<1>(state) = py;
      //      get<2>(state) = vx; get<3>(state) = vy;
      get<0>(state) = pos_x; get<1>(state) = pos_y;
      get<2>(state) = vel_x; get<3>(state) = vel_y;
      auto h = heading;
      heading = msg->robotinfo.at(i).heading.theta;
      heading_rate = heading - h;
      // ROS_INFO("robot info(%f,%f,%f,%f)", px, py, vx, vy);
    }
    else {
      decay_t<decltype(obs.front())> s;
      x(s) = pos_x;
      y(s) = pos_y;
      vx(s) = vel_x;
      vy(s) = vel_y;
      obs.push_back(s);
    }
  }

#if 0
  // select nearest obs, returning index
  auto select_obs = [](auto &pos, auto &obs, auto &exclude) {
    auto id = 0;
    auto dis = 1e5;
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
  std::vector<size_t> o_ids;
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
      o_ids.push_back(i);
    }
  }
  if(ids.size() > 0) {

  }
#endif
  last_recv = time;
  //  ROS_INFO("f : %f", f);
  //  for(auto o : obs)
  //    ROS_INFO("(%f,%f,%f,%f)", o(0), o(1), o(2), o(3));
  // publish tf
  this->publishTF();
}

void RobotSubscriber::publishTF()
{
  transform.setOrigin(tf::Vector3(get<0>(state), get<1>(state), 0.0));
  tf::Quaternion q;
  q.setRPY(0.0,0.0,heading);
  transform.setRotation(q);
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", tf_name));
}
