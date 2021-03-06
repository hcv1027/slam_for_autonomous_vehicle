#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_SENSOR_DATA_IMU_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_SENSOR_DATA_IMU_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

namespace slam_for_autonomous_vehicle {

struct Imu {
  struct Orientation {
    double w = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct AngularVel {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct LinearAcc {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  double time_stamp = 0.0;
  Orientation orientation;
  AngularVel angular_vel;
  LinearAcc linear_acc;

 public:
  Imu() = default;

  Imu(const sensor_msgs::Imu& data) {
    time_stamp = data.header.stamp.toSec();
    orientation.w = data.orientation.w;
    orientation.x = data.orientation.x;
    orientation.y = data.orientation.y;
    orientation.z = data.orientation.z;
    angular_vel.x = data.angular_velocity.x;
    angular_vel.y = data.angular_velocity.y;
    angular_vel.z = data.angular_velocity.z;
    linear_acc.x = data.linear_acceleration.x;
    linear_acc.y = data.linear_acceleration.y;
    linear_acc.z = data.linear_acceleration.z;
  }

  Imu(const Imu& imu) {
    time_stamp = imu.time_stamp;
    orientation = imu.orientation;
    angular_vel = imu.angular_vel;
    linear_acc = imu.linear_acc;
  }

  ~Imu() = default;

  Eigen::Matrix3f GetOrientationMatrix() {
    Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y,
                         orientation.z);
    Eigen::Matrix3f matrix = q.matrix().cast<float>();
    return matrix;
  }
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_SENSOR_DATA_IMU_H_