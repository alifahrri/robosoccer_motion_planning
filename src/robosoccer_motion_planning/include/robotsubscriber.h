#ifndef ROBOTSUBSCRIBER_H
#define ROBOTSUBSCRIBER_H

#include <queue>
#include <vector>
#include <ros/ros.h>
#include <nubot_common/OminiVisionInfo.h>
#include <tf/transform_broadcaster.h>

class RobotSubscriber
{
public:
  // struct State : std::tuple<double,double,double,double>
  struct State : std::array<double,4>
  {
    const double& operator()(size_t i) const
    {
      return (*this)[i];
    }
    double& operator()(size_t i)
    {
      return (*this)[i];
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
  void rival_callback(const nubot_common::OminiVisionInfo::ConstPtr &msg);
  size_t numPublishers() { return sub.getNumPublishers(); }
  void publishTF();
  const auto& getState() const { return state; }
  auto getObstacles() const {
    decltype(obs) ret;
    ret.insert(ret.begin(), obs.begin(), obs.end());
    ret.insert(ret.end(), robs.begin(), robs.end());
    return ret;
  }
  const auto getHeading() const { return std::make_pair(heading, heading_rate); }
  const auto& getTF() const { return transform; }
  const auto& getName() const { return tf_name; }
private:
  tf::Transform transform;
  tf::TransformBroadcaster tf_broadcaster;
  ros::Subscriber sub;
  ros::Subscriber rsub; // subsriber for rival
  std::vector<State> obs; // friend
  std::vector<State> robs; // foe
  std::vector<State> obs_p0;
  std::vector<std::vector<State>> obs_history;
  std::vector<std::vector<ros::Time>> time_history;
  State state;
  double heading;
  double heading_rate;
  size_t id;
  size_t n_obstacles = 9;
  ros::Time last_recv;
  ros::Time vel_time;
  double f;
  std::string tf_name;
};

#endif // ROBOTSUBSCRIBER_H
