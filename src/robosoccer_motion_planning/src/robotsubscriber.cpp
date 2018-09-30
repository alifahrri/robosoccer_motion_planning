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
  obs.clear();
  for(size_t i=0; i<msg->obstacleinfo.pos.size(); i++) {
    Pos o;
    auto pos = msg->obstacleinfo.pos.at(i);
    get<0>(o) = pos.x/100.0;
    get<1>(o) = pos.y/100.0;
    obs.push_back(o);
  }
  // ROS_INFO("pos(%f,%f,%f,%f), obs_size : %d", state(0), state(1), state(2), state(3), obs.size());
}
