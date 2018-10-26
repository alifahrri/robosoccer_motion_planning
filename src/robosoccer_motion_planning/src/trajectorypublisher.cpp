#include "trajectorypublisher.h"

TrajectoryPublisher::TrajectoryPublisher(ros::NodeHandle &node, std::string pos_topic, std::string vel_topic)
{
  ppub = node.advertise<nav_msgs::Path>(pos_topic, 3);
  vpub = node.advertise<nav_msgs::Path>(vel_topic, 3);
}

void TrajectoryPublisher::publish()
{
  ppub.publish(pos);
  vpub.publish(vel);
}

