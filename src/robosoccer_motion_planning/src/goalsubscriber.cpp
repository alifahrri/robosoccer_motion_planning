#include "goalsubscriber.h"

GoalSubscriber::GoalSubscriber(ros::NodeHandle &node, std::string topic)
{
  sub = node.subscribe(topic, 3, &GoalSubscriber::callback, this);
}

void GoalSubscriber::callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  goal.header = msg->header;
  goal.pose = msg->pose;
}
