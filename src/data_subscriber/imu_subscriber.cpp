#include "data_subscriber/imu_subscriber.h"
#include <string>

using std::string;
// using slam_for_autonomous_vehicle::Imu;

namespace slam_for_autonomous_vehicle {
ImuSubscriber::ImuSubscriber(ros::NodeHandle& nh, size_t buffer_size) {
  nh_ = nh;
  string topic_name = nh_.param(string("imu_topic"), string("imu_topic_name"));
  ROS_INFO_STREAM("imu_topic: " << topic_name);
  subscriber_ = nh_.subscribe(topic_name, buffer_size,
                              &ImuSubscriber::msg_callback, this);
}

void ImuSubscriber::msg_callback(const sensor_msgs::Imu& msg) {
  Imu imu(msg);
  data_.push_back(imu);
  //   ROS_INFO_STREAM("Receive imu msg");
}
}