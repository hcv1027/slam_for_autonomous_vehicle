#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_DATA_SUBSCRIBER_IMU_SUBSCRIBER_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_DATA_SUBSCRIBER_IMU_SUBSCRIBER_H_

#include "sensor_data/imu.h"
#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace slam_for_autonomous_vehicle {

class ImuSubscriber {
public:
  ImuSubscriber(ros::NodeHandle &nh, size_t buffer_size);
  ImuSubscriber() = default;
  ~ImuSubscriber() = default;

  void get_data(std::deque<Imu> &data_deque);

private:
  void msg_callback(const sensor_msgs::Imu &msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<Imu> data_;
};

} // namespace slam_for_autonomous_vehicle

#endif // SLAM_FOR_AUTONOMOUS_VEHICLE_DATA_SUBSCRIBER_IMU_SUBSCRIBER_H_