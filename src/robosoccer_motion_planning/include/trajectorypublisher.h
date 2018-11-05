#ifndef TRAJECTORYPUBLISHER_H
#define TRAJECTORYPUBLISHER_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include "trajectoryhelper.hpp"
#include "elements.hpp"

template <std::size_t n>
struct choice : choice<n-1>
{};

template <>
struct choice<0> {};

class TrajectoryPublisher
{
public:
  TrajectoryPublisher(ros::NodeHandle &node, std::string pos_topic, std::string vel_topic, std::string vel_frame = std::string("map"), std::string frame = std::string("map"), std::string name = std::string("trajectory_tf"));
  // this will read from stored nav_msgs
  auto get(auto &state, auto &heading, auto tref);
  // this would set the nav_msgs
  auto set(const auto &array_like, auto &get_time, auto &get_pos, auto &get_vel);
  auto set(const auto &pos_array, const auto &vel_array, auto &get_time, auto &get_pos, auto &get_vel);
  auto set_pose2d(const auto &array_like, auto &get_time, auto &get_pos, auto &get_vel, auto &angle_getter_fn);
  // this will set transform
  auto set_initial(const auto &origin, const auto &yaw);
  void publish();
private:
  inline void reset();
private:
  ros::Publisher ppub;
  ros::Publisher vpub;
  nav_msgs::Path pos;
  nav_msgs::Path vel;
  // a name for transform broadcaster
  std::string tf_name;
  // frame name for nav_msgs
  std::string vel_frame;
  std::string pos_frame;
  ros::Time tf_stamp;
  tf::Transform transform;
  tf::TransformBroadcaster tf_broadcaster;

private: // internal use only

  struct ct_caller
  {
    auto operator ()(choice<2>, const auto &stamp, auto t) -> decltype(stamp.toSec() < t.toSec()){
      return stamp.toSec() > t.toSec();
    }
    auto operator ()(choice<1>, const auto &stamp, auto t) -> decltype(stamp.toSec() < t){
      return stamp.toSec() > t;
    }
    auto operator ()(choice<0>, const auto &stamp, auto t) -> decltype(stamp < t){
      return stamp > t;
    }
    auto operator ()(const auto &stamp, auto t) {
      return this->operator ()(choice<2>{}, stamp, t);
    }
  };
  struct heading_caller
  {
    auto copy(const auto &src, auto &dst) -> decltype(dst = src, void()){
      dst = src;
    }
    auto copy(const auto &src, auto &dst) -> decltype(dst(0) = std::get<0>(src), void()){
      dst(0) = std::get<0>(src);
      dst(1) = std::get<1>(src);
    }
    auto copy(const auto &src, auto &dst) -> decltype(std::get<0>(dst) = src(0), void()){
      std::get<0>(dst) = src(0);
      std::get<1>(dst) = src(1);
    }
  };
};

auto TrajectoryPublisher::get(auto &state, auto &heading, auto tref)
{
  for(size_t i=0; i<pos.poses.size(); i++) {
    ct_caller check_time;
    heading_caller heading_op;
    if(check_time(pos.poses.at(i).header.stamp, tref)) {
      // assuming state has () operator
      state(0) = pos.poses.at(i).pose.position.x;
      state(1) = pos.poses.at(i).pose.position.y;
      state(2) = vel.poses.at(i).pose.position.x;
      state(3) = vel.poses.at(i).pose.position.y;
      auto po = pos.poses.at(i).pose.orientation;
      auto vo = vel.poses.at(i).pose.orientation;
      auto hp = std::make_pair(tf::getYaw(po),tf::getYaw(vo));
      heading_op.copy(hp,heading);
      break;
    }
  }
}

// this version also set angle value
// note that angle adapytor should take angle_array value and two doubles represent angle pos and vel
auto TrajectoryPublisher::set_pose2d(const auto &array_like, auto &get_time, auto &get_pos, auto &get_vel, auto &angle_getter_fn)
{
  this->set(array_like, get_time, get_pos, get_vel);
  // here, we assign angles to path message given angle_array and the adaptor
  for(size_t i=0; i<pos.poses.size(); i++) {
    auto &p = pos.poses.at(i);
    auto &v = vel.poses.at(i);
    auto w = .0;
    auto dw = .0;
    // at this point, angle_adaptor should set w and dw given angle value
    angle_getter_fn(i, w, dw);
    tf::Quaternion q, dq;
    q.setRPY(0, 0, w);
    dq.setRPY(0, 0, dw);
    geometry_msgs::Quaternion quat, quat_rate;
    tf::quaternionTFToMsg(q, quat);
    tf::quaternionTFToMsg(dq, quat_rate);
    p.pose.orientation = quat;
    v.pose.orientation = quat_rate;
  }
}

auto TrajectoryPublisher::set_initial(const auto &origin, const auto &yaw)
{
  tf_stamp = ros::Time::now();
  transform.setOrigin(tf::Vector3(x(origin), y(origin), 0.0));
  tf::Quaternion q;
  q.setRPY(.0,.0,yaw);
  transform.setRotation(q);
}

// set pos and vel path message
// this version takes unified source pos and vel array
auto TrajectoryPublisher::set(const auto &array_like, auto &get_time, auto &get_pos, auto &get_vel)
{
  size_t i=0;
  reset();
  for(const auto &s : array_like) {
    geometry_msgs::PoseStamped p, v;
    assign_pose(p, get_pos_vel(get_pos, s, i));
    assign_pose(v, get_pos_vel(get_vel, s, i));
    auto t = get_time_call(get_time, s, i);
    p.header.seq = i;
    p.header.stamp = t;
    v.header.seq = i;
    v.header.stamp = t;
    pos.poses.push_back(p);
    vel.poses.push_back(v);
    i += 1;
  }
}

// set pos and vel path message
// this version separate the source pos and vel array
auto TrajectoryPublisher::set(const auto &pos_array, const auto &vel_array, auto &get_time, auto &get_pos, auto &get_vel)
{
  size_t i=0;
  reset();
  for(const auto &s : pos_array) {
    geometry_msgs::PoseStamped p;
    assign_pose(p, get_pos_vel(get_pos, s, i));
    auto t = get_time_call(get_time, s, i);
    p.header.seq = i;
    p.header.stamp = t;
    pos.poses.push_back(p);
    i += 1;
  }
  i=0;
  for(const auto &s : vel_array) {
    geometry_msgs::PoseStamped v;
    assign_pose(v, get_pos_vel(get_vel, s, i));
    auto t = get_time_call(get_time, s, i);
    v.header.seq = i;
    v.header.stamp = t;
    vel.poses.push_back(v);
    i += 1;
  }
}

void TrajectoryPublisher::reset()
{
  vel.header.frame_id = vel_frame;
  pos.header.frame_id = pos_frame;
  vel.header.seq++;
  pos.header.seq++;
  vel.header.stamp = ros::Time::now();
  pos.header.stamp = ros::Time::now();
  pos.poses.clear();
  vel.poses.clear();
}

#endif // TRAJECTORYPUBLISHER_H
