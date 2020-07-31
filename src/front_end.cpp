#include "front_end.h"

using std::string;

namespace slam_for_autonomous_vehicle {
FrontEnd::FrontEnd(ros::NodeHandle &nh) : nh_(nh) {
  float leaf_size = nh_.param(string("leaf_size"), 0.2);
  cloud_filter_.setLeafSize(leaf_size, leaf_size, leaf_size); // Unit: meter
}

void FrontEnd::Update(const Cloud &cloud) {
  Cloud filteded_cloud;
  cloud_filter_.setInputCloud(cloud.cloud_ptr);
  cloud_filter_.filter(*(filteded_cloud.cloud_ptr));
}

void FrontEnd::AddKeyFrame() {}
}