#include "trajectorypublisher.h"

TrajectoryPublisher::TrajectoryPublisher(ros::NodeHandle &node, std::string pos_topic, std::string vel_topic, std::string vel_frame, std::string pos_frame, std::string name)
  : vel_frame(vel_frame), pos_frame(pos_frame), tf_name(name)
{
  ppub = node.advertise<nav_msgs::Path>(pos_topic, 3);
  vpub = node.advertise<nav_msgs::Path>(vel_topic, 3);
}

void TrajectoryPublisher::publish()
{
  ppub.publish(pos);
  vpub.publish(vel);
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, tf_stamp, pos_frame, tf_name));
}
