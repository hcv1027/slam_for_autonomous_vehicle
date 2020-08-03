#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_FRONT_END_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_FRONT_END_H_

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Dense>
#include <deque>
#include "sensor_data/cloud.h"
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

  Eigen::Matrix4f Update(const Cloud &cloud);

  void SetInitPose(const Eigen::Matrix4f &init_pose);

 private:
  void AddKeyFrame(Frame &keyframe);

 private:
  ros::NodeHandle nh_;
  pcl::VoxelGrid<pcl::PointXYZ> cloud_filter_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

  Eigen::Matrix4f curr_pose_;
  Eigen::Matrix4f last_pose_;
  Eigen::Matrix4f predict_pose_;

  std::deque<Frame> local_keyframe_;
  Cloud local_map_;
  std::deque<Frame> global_keyframe_;
  Cloud global_map_;
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_FRONT_END_H_