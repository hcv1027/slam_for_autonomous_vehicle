#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_SENSOR_DATA_CLOUD_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_SENSOR_DATA_CLOUD_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace slam_for_autonomous_vehicle {

struct Cloud {
  using POINT = pcl::PointXYZ;
  using CLOUD = pcl::PointCloud<POINT>;
  using CLOUD_PTR = CLOUD::Ptr;

  double time_stamp_;
  CLOUD data_;

  Cloud() = default;

  Cloud(const sensor_msgs::PointCloud2& data) {
    time_stamp_ = data.header.stamp.toSec();
    pcl::fromROSMsg(data, this->data_);
  }

  Cloud(const Cloud& cloud) {
    data_ = cloud.data_;
    time_stamp_ = cloud.time_stamp_;
  }

  ~Cloud() = default;
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_SENSOR_DATA_CLOUD_H_