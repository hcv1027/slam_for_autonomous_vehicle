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
  using CLOUD_PTR = CLOUD::Ptr;  // boost::shared_ptr

  double time_stamp;
  CLOUD_PTR cloud_ptr;

  Cloud() : cloud_ptr(new CLOUD), time_stamp(0.0) {}

  Cloud(const sensor_msgs::PointCloud2 &msg) : cloud_ptr(new CLOUD) {
    time_stamp = msg.header.stamp.toSec();
    pcl::fromROSMsg(msg, *this->cloud_ptr);
  }

  Cloud(const Cloud &cloud) {
    cloud_ptr = cloud.cloud_ptr;
    time_stamp = cloud.time_stamp;
  }

  ~Cloud() = default;

  void reset() { cloud_ptr = CLOUD_PTR(new CLOUD); }
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_SENSOR_DATA_CLOUD_H_