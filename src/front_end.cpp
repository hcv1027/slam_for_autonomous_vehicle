#include "front_end.h"

using std::string;

namespace slam_for_autonomous_vehicle {
FrontEnd::FrontEnd(ros::NodeHandle &nh) : nh_(nh) {
  float leaf_size = nh_.param(string("leaf_size"), 0.2);
  cloud_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);  // Unit: meter

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
  Cloud filteded_cloud;
  cloud_filter_.setInputCloud(cloud.cloud_ptr);
  cloud_filter_.filter(*(filteded_cloud.cloud_ptr));

  // Add first key frame
  if (local_keyframe_.size() == 0) {
    return AddKeyFrame(filteded_cloud);
  }

  // Setting point cloud to be aligned.
  Cloud output_cloud;
  ndt_.setInputSource(filteded_cloud.cloud_ptr);
  ndt_.align(*output_cloud.cloud_ptr, predict_pose_);
  Eigen::Matrix4f current_pose = ndt_.getFinalTransformation();
  return current_pose;
}

Eigen::Matrix4f FrontEnd::AddKeyFrame(const Cloud &cloud) {
  // Add to local keyframe
  Frame curr_frame;
  curr_frame.cloud = cloud;
  curr_frame.pose = Eigen::Matrix4f::Identity();

  // Update local keyframe and local map
  local_keyframe_.push_back(curr_frame);
  if (local_keyframe_.size() > 20) {
    local_keyframe_.pop_front();
  }
  local_map_.reset();
  for (Frame &keyframe : local_keyframe_) {
    *local_map_.cloud_ptr += *keyframe.cloud.cloud_ptr;
  }
  // Setting point cloud to be aligned to.
  ndt_.setInputTarget(local_map_.cloud_ptr);

  // Update global keyfram and global map

  return curr_frame.pose;
}
}