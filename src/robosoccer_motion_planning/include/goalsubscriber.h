#ifndef GOALSUBSCRIBER_H
#define GOALSUBSCRIBER_H

#include <type_traits>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include "goalhelper.hpp"

class GoalSubscriber
{
public:
  GoalSubscriber(ros::NodeHandle &node, std::string topic);
  void setGoal(auto &pose);
  void getGoal(auto &position);
  auto getGoal();
  auto getYaw();
  auto goalChanged() { return goal_changed; }
private:
  void set_time(auto &var);
  void callback(const geometry_msgs::PoseStampedConstPtr &msg);
private:
  ros::Subscriber sub;
  geometry_msgs::PoseStamped goal;
  bool goal_changed = true;
};

void GoalSubscriber::setGoal(auto &position)
{
  x(goal.pose.position) = x(position);
  y(goal.pose.position) = y(position);
}

void GoalSubscriber::getGoal(auto &pos)
{
  set_xyz(goal.pose.position, pos);
  goal_changed = false;
}

// this version simply return xyz and rpy
auto GoalSubscriber::getGoal()
{
  tf::Quaternion q;
  goal_changed = false;
  tf::quaternionMsgToTF(goal.pose.orientation, q);
  return std::make_tuple(goal.pose.position.x, goal.pose.position.y, tf::getYaw(q));
}

auto GoalSubscriber::getYaw()
{
  tf::Quaternion q;
  goal_changed = false;
  tf::quaternionMsgToTF(goal.pose.orientation, q);
  return tf::getYaw(q);
}

#endif // GOALSUBSCRIBER_H
