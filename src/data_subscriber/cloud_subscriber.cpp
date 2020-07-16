#include "data_subscriber/cloud_subscriber.h"
#include <string>

using std::string;
// using slam_for_autonomous_vehicle::Cloud;

namespace slam_for_autonomous_vehicle {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, size_t buffer_size) {
  nh_ = nh;
  string topic_name =
      nh_.param(string("cloud_topic"), string("cloud_topic_name"));
  ROS_INFO_STREAM("cloud_topic: " << topic_name);
  subscriber_ = nh_.subscribe(topic_name, buffer_size,
                              &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2& msg) {
  Cloud cloud(msg);
  data_.push_back(cloud);
  ROS_INFO_STREAM("Receive cloud msg");
}
}