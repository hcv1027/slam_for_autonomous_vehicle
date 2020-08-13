#include "publisher/cloud_publisher.h"
#include <pcl_conversions/pcl_conversions.h>

namespace slam_for_autonomous_vehicle {

CloudPublisher::CloudPublisher(ros::NodeHandle &nh,
                               const std::string &topic_name,
                               const std::string &frame_id) {
  publisher_ = nh.advertise<sensor_msgs::PointCloud2>("topic_name", 1000);

  // Initialize cloud message header
  msg_.header.seq = 0;
  msg_.header.stamp = ros::Time::now();
  msg_.header.frame_id = frame_id;
}

void CloudPublisher::publish(const Cloud &cloud) {
  msg_.header.seq++;
  msg_.header.stamp = ros::Time::now();
  pcl::toROSMsg(*cloud.cloud_ptr, msg_);

  publisher_.publish(msg_);
}
}  // namespace slam_for_autonomous_vehicle