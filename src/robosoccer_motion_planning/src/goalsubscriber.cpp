#include "goalsubscriber.h"

GoalSubscriber::GoalSubscriber(ros::NodeHandle &node, std::string topic)
{
  sub = node.subscribe(topic, 3, &GoalSubscriber::callback, this);
  tf::Quaternion q;
  q.setRPY(0.0,0.0,0.0);
  tf::quaternionTFToMsg(q, goal.pose.orientation);
}

void GoalSubscriber::callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  goal_changed = true;
  goal.header = msg->header;
  goal.pose = msg->pose;
}
