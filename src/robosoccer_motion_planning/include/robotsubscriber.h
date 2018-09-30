#ifndef ROBOTSUBSCRIBER_H
#define ROBOTSUBSCRIBER_H

#include <vector>
#include <ros/ros.h>
#include <nubot_common/OminiVisionInfo.h>

class RobotSubscriber
{
public:
  struct State : std::tuple<double,double,double,double>
  {
    double operator()(size_t i) const {
      switch (i) {
      case 0:
        return std::get<0>(*this);
      case 1:
        return std::get<1>(*this);
      case 2:
        return std::get<2>(*this);
      case 3:
        return std::get<3>(*this);
      }
    }
  };

  struct Pos : public std::tuple<double,double> {
    double operator()(size_t i) const {
      switch (i) {
      case 0:
        return std::get<0>(*this);
      case 1:
        return std::get<1>(*this);
      }
    }
  };

  enum TEAM {
    NUBOT,
    RIVAL
  };

public:
  RobotSubscriber(ros::NodeHandle &node, size_t robot_id, TEAM team);
  void callback(const nubot_common::OminiVisionInfo::ConstPtr &msg);
  size_t numPublishers() { return sub.getNumPublishers(); }
  const State& getState() const { return state; }
  const std::vector<Pos>& getObstacles() const { return obs; }
private:
  ros::Subscriber sub;
  std::vector<Pos> obs;
  State state;
  double heading;
  size_t id;
};

#endif // ROBOTSUBSCRIBER_H
