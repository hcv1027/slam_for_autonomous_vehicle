#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_PUBLISHER_CLOUD_PUBLISHER_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_PUBLISHER_CLOUD_PUBLISHER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <string>
#include "sensor_data/cloud.h"

namespace slam_for_autonomous_vehicle {

class CloudPublisher {
 private:
  ros::Publisher publisher_;
  sensor_msgs::PointCloud2 msg_;

 public:
  CloudPublisher(ros::NodeHandle &nh, const std::string &topic_name,
                 const std::string &frame_id);
  CloudPublisher() = default;
  ~CloudPublisher() = default;

  void publish(const Cloud &cloud);
};

/* CloudPublisher::CloudPublisher()
{
}

CloudPublisher::~CloudPublisher()
{
} */

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_PUBLISHER_CLOUD_PUBLISHER_H_