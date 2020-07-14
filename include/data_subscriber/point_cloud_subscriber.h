#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE__POINT_CLOUD_SUBSCRIBER_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_POINT_CLOUD_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <deque>

namespace slam_for_autonomous_vehicle {

class PointCloudSubscriber {
 public:
  PointCloudSubscriber(ros::NodeHandle& nh, size_t buffer_size);
  PointCloudSubscriber() = default;
  ~PointCloudSubscriber();

 private:
  void msg_callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<sensor_msgs::PointCloud2> data_;
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_POINT_CLOUD_SUBSCRIBER_H_