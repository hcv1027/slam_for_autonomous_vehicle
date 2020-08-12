#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_PUBLISHER_ODOMETRT_PUBLISHER_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_PUBLISHER_ODOMETRT_PUBLISHER_H_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <string>

namespace slam_for_autonomous_vehicle {

class OdometryPublisher {
 private:
  ros::Publisher publisher_;
  nav_msgs::Odometry msg_;

 public:
  OdometryPublisher(ros::NodeHandle &nh, const std::string &topic_name,
                    const std::string &base_frame,
                    const std::string &child_frame);
  OdometryPublisher() = default;
  ~OdometryPublisher() = default;

  void publish(const Eigen::Matrix4f &transform);
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_PUBLISHER_ODOMETRT_PUBLISHER_H_