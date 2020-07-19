#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_DATA_SUBSCRIBER_GNSS_SUBSCRIBER_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_DATA_SUBSCRIBER_GNSS_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <deque>
#include "sensor_data/gnss.h"

namespace slam_for_autonomous_vehicle {

class GnssSubscriber {
 public:
  GnssSubscriber(ros::NodeHandle& nh, size_t buffer_size);
  GnssSubscriber() = default;
  ~GnssSubscriber() = default;

 private:
  void msg_callback(const sensor_msgs::NavSatFix& msg);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<Gnss> data_;
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_DATA_SUBSCRIBER_GNSS_SUBSCRIBER_H_