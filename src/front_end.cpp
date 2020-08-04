#include "front_end.h"
#include <pcl/common/transforms.h>
#include <cmath>

using std::string;
using std::sqrt;
using std::pow;

namespace slam_for_autonomous_vehicle {
FrontEnd::FrontEnd(ros::NodeHandle &nh) : nh_(nh) {
  float leaf_size = nh_.param(string("leaf_size"), 0.2);
  cloud_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);  // Unit: meter

  curr_pose_ = Eigen::Matrix4f::Identity();
  last_pose_ = Eigen::Matrix4f::Identity();
  predict_pose_ = Eigen::Matrix4f::Identity();

  // Initialize ndt
  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt_.setTransformationEpsilon(0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt_.setStepSize(0.1);
  // Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt_.setResolution(1.0);
  // Setting max number of registration iterations.
  ndt_.setMaximumIterations(35);
}

Eigen::Matrix4f FrontEnd::Update(const Cloud &cloud) {
  std::vector<int> indices;
  Cloud no_nan_cloud;
  Cloud filteded_cloud;
  pcl::removeNaNFromPointCloud(*cloud.cloud_ptr, *no_nan_cloud.cloud_ptr,
                               indices);
  cloud_filter_.setInputCloud(no_nan_cloud.cloud_ptr);
  cloud_filter_.filter(*filteded_cloud.cloud_ptr);

  // Add first key frame
  if (local_keyframe_.size() == 0) {
    Frame keyframe;
    keyframe.cloud = cloud;
    keyframe.pose = Eigen::Matrix4f::Identity();
    AddKeyFrame(keyframe);
    return keyframe.pose;
  }

  // Setting point cloud to be aligned.
  Cloud output_cloud;
  ndt_.setInputSource(filteded_cloud.cloud_ptr);
  ndt_.align(*output_cloud.cloud_ptr, predict_pose_);
  curr_pose_ = ndt_.getFinalTransformation();

  // Prdtict next pose formula: next_pose = curr_pose * step_pose
  Eigen::Matrix4f step_pose = last_pose_.inverse() * curr_pose_;
  predict_pose_ = curr_pose_ * step_pose;
  last_pose_ = curr_pose_;

  // Add new keyframe
  Frame &last_keyframe = local_keyframe_.back();
  double dist = pow(curr_pose_(0, 3) - last_keyframe.pose(0, 3), 2) +
                pow(curr_pose_(1, 3) - last_keyframe.pose(1, 3), 2) +
                pow(curr_pose_(2, 3) - last_keyframe.pose(2, 3), 2);
  if (dist > 2.0) {
    Frame keyframe;
    keyframe.cloud = cloud;
    keyframe.pose = curr_pose_;
    AddKeyFrame(keyframe);
  }

  return curr_pose_;
}

void FrontEnd::AddKeyFrame(Frame &keyframe) {
  // Update local keyframe and local map
  local_keyframe_.push_back(keyframe);
  if (local_keyframe_.size() > 20) {
    local_keyframe_.pop_front();
  }
  local_map_.reset();
  for (Frame &keyframe : local_keyframe_) {
    Cloud transformed_cloud;
    pcl::transformPointCloud(*keyframe.cloud.cloud_ptr,
                             *transformed_cloud.cloud_ptr, keyframe.pose);
    *local_map_.cloud_ptr += *transformed_cloud.cloud_ptr;
  }

  // Setting point cloud to be aligned to.
  Cloud filtered_local_map;
  cloud_filter_.setInputCloud(local_map_.cloud_ptr);
  cloud_filter_.filter(*filtered_local_map.cloud_ptr);
  ndt_.setInputTarget(filtered_local_map.cloud_ptr);

  // Update global keyfram and global map
}

void FrontEnd::SetInitPose(const Eigen::Matrix4f &init_pose) {
  init_pose_ = init_pose;
  curr_pose_ = init_pose;
  last_pose_ = init_pose;
  predict_pose_ = init_pose;
}
}