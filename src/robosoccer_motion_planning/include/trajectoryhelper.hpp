#ifndef TRAJECTORYHELPER_HPP
#define TRAJECTORYHELPER_HPP

#include <geometry_msgs/PoseStamped.h>

auto get_time_call(auto &time, const auto &s, size_t i) -> decltype(time(s))
{
  return time(s);
}

auto get_time_call(auto &time, const auto &s, size_t i) -> decltype(time(i))
{
  return time(i);
}

auto get_pos_vel(auto &pos, const auto &s, size_t i) -> decltype(pos(s))
{
  return pos(s);
}

auto get_pos_vel(auto &pos, const auto &s, size_t i) -> decltype(pos(i))
{
  return pos(i);
}

auto assign_pose(geometry_msgs::PoseStamped &pose, const auto& p) -> decltype(p.x, p.y, void())
{
  pose.pose.position.x = p.x;
  pose.pose.position.y = p.y;
}

auto assign_pose(geometry_msgs::PoseStamped &pose, const auto& p) -> decltype(p[0], void())
{
  pose.pose.position.x = p[0];
  pose.pose.position.y = p[1];
}

auto assign_pose(geometry_msgs::PoseStamped &pose, const auto &p) -> decltype(p(0), void())
{
  pose.pose.position.x = p(0);
  pose.pose.position.y = p(1);
}

#endif // TRAJECTORYHELPER_HPP
