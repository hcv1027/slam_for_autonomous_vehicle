#include "publisher/odometry_publisher.h"

namespace slam_for_autonomous_vehicle {

OdometryPublisher::OdometryPublisher(ros::NodeHandle &nh,
                                     const std::string &topic_name,
                                     const std::string &base_frame,
                                     const std::string &child_frame) {
  publisher_ = nh.advertise<nav_msgs::Odometry>("topic_name", 1000);
  // Initialize odometry message header
  msg_.header.seq = 0;
  msg_.header.stamp = ros::Time::now();
  msg_.header.frame_id = base_frame;
  msg_.child_frame_id = child_frame;
}

void OdometryPublisher::publish(const Eigen::Matrix4f &transform) {
  msg_.pose.pose.position.x = transform(0, 3);
  msg_.pose.pose.position.y = transform(1, 3);
  msg_.pose.pose.position.z = transform(2, 3);

  Eigen::Quaternionf q;
  q = transform.block<3, 3>(0, 0);
  msg_.pose.pose.orientation.x = q.x();
  msg_.pose.pose.orientation.y = q.y();
  msg_.pose.pose.orientation.z = q.z();
  msg_.pose.pose.orientation.w = q.w();

  publisher_.publish(msg_);
  msg_.header.seq++;
}
}  // namespace slam_for_autonomous_vehicle