#include "data_subscriber/point_cloud_subscriber.h"
#include <string>

using std::string;

namespace slam_for_autonomous_vehicle {

PointCloudSubscriber::PointCloudSubscriber(ros::NodeHandle& nh,
                                           size_t buffer_size) {
  nh_ = nh;
  string topic_name = "";
  nh_.param("cloud_topic", topic_name, "/kitti/velo/pointcloud");
  subscriber_ = nh_.subscribe(topic_name, buffer_size,
                              &PointCloudSubscriber::msg_callback, this);
}

void PointCloudSubscriber::msg_callback(const sensor_msgs::PointCloud2& msg) {
  data_.push_back(msg);
}
}