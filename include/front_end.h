#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_FRONT_END_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_FRONT_END_H_

#include "sensor_data/cloud.h"
#include <Eigen/Dense>
#include <deque>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>

namespace slam_for_autonomous_vehicle {

class FrontEnd {
  struct Frame {
    Eigen::Matrix4f pose;
    Cloud cloud;
  };

public:
  FrontEnd(ros::NodeHandle &nh);
  FrontEnd() = default;
  ~FrontEnd() = default;

  void Update(const Cloud &cloud);
  void AddKeyFrame();

private:
private:
  ros::NodeHandle nh_;
  pcl::VoxelGrid<pcl::PointXYZ> cloud_filter_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  std::deque<Frame> local_map_;
  std::deque<Frame> global_map_;
};

} // namespace slam_for_autonomous_vehicle

#endif // SLAM_FOR_AUTONOMOUS_VEHICLE_FRONT_END_H_