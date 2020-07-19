#include "data_subscriber/gnss_subscriber.h"
#include <string>

using std::string;

namespace slam_for_autonomous_vehicle {
GnssSubscriber::GnssSubscriber(ros::NodeHandle& nh, size_t buffer_size) {
  nh_ = nh;
  string topic_name =
      nh_.param(string("gnss_topic"), string("gnss_topic_name"));
  //   ROS_INFO_STREAM("gnss_topic: " << topic_name);
  subscriber_ = nh_.subscribe(topic_name, buffer_size,
                              &GnssSubscriber::msg_callback, this);
}

void GnssSubscriber::msg_callback(const sensor_msgs::NavSatFix& msg) {
  Gnss gnss(msg);
  data_.push_back(gnss);
  ROS_INFO_STREAM("Receive gnss msg");
}
}