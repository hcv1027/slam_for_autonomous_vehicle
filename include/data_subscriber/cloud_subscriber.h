#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_DATA_SUBSCRIBER_CLOUD_SUBSCRIBER_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_DATA_SUBSCRIBER_CLOUD_SUBSCRIBER_H_

#include "sensor_data/cloud.h"
#include <deque>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace slam_for_autonomous_vehicle {

class CloudSubscriber {
public:
  CloudSubscriber(ros::NodeHandle &nh, size_t buffer_size);
  CloudSubscriber() = default;
  ~CloudSubscriber() = default;

  void get_data(std::deque<Cloud> &data_deque);

private:
  void msg_callback(const sensor_msgs::PointCloud2 &msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<Cloud> data_;
};

} // namespace slam_for_autonomous_vehicle

#endif // SLAM_FOR_AUTONOMOUS_VEHICLE_DATA_SUBSCRIBER_CLOUD_SUBSCRIBER_H_