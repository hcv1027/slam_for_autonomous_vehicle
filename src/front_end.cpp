#include "front_end.h"
#include <pcl/common/transforms.h>
#include <ros/console.h>
#include <cmath>
#include "models/registration/ndt_registration.h"
#include "models/voxel_filter/approximate_voxel_filter.h"

using std::pow;
using std::sqrt;
using std::string;

namespace slam_for_autonomous_vehicle {
FrontEnd::FrontEnd(ros::NodeHandle &nh) : nh_(nh) {
  float leaf_size = nh_.param(string("cloud_filter_leaf_size"), 0.5);
  cloud_filter_ptr_ = std::make_shared<ApproximateVoxelFilter>(leaf_size);

  leaf_size = nh_.param(string("local_map_filter_leaf_size"), 2.0);
  local_map_filter_ptr_ = std::make_shared<ApproximateVoxelFilter>(leaf_size);

  last_pose_ = Eigen::Matrix4f::Identity();
  predict_pose_ = Eigen::Matrix4f::Identity();

  // Initialize ndt registration
  registration_ptr_ = std::make_shared<NdtRegistration>();
}

FrontEnd::Frame FrontEnd::Update(const Cloud &cloud) {
  std::vector<int> indices;
  Cloud no_nan_cloud;
  Cloud filtered_cloud;
  pcl::removeNaNFromPointCloud(*(cloud.cloud_ptr), *(no_nan_cloud.cloud_ptr),
                               indices);
  cloud_filter_ptr_->Filter(no_nan_cloud, filtered_cloud);
  ROS_INFO_STREAM("Filter input cloud: " << filtered_cloud.cloud_ptr->size());

  // Add first key frame
  if (local_keyframe_.size() == 0) {
    curr_frame_.cloud = no_nan_cloud;
    curr_frame_.pose = Eigen::Matrix4f::Identity();
    AddKeyFrame(curr_frame_);
    return curr_frame_;
  }

  // Scan match
  Cloud output_cloud;
  Eigen::Matrix4f output_pose = Eigen::Matrix4f::Identity();
  registration_ptr_->ScanMatch(filtered_cloud, predict_pose_, output_cloud,
                               output_pose);
  curr_frame_.cloud = output_cloud;
  curr_frame_.pose = output_pose;
  // ROS_INFO_STREAM("cloud_pose: " << curr_frame_.pose);

  // Predict next pose formula: next_pose = curr_pose * step_pose
  Eigen::Matrix4f step_pose = last_pose_.inverse() * curr_frame_.pose;
  predict_pose_ = curr_frame_.pose * step_pose;
  last_pose_ = curr_frame_.pose;

  // Add new keyframe
  Frame &last_keyframe = local_keyframe_.back();
  double dist = pow(curr_frame_.pose(0, 3) - last_keyframe.pose(0, 3), 2) +
                pow(curr_frame_.pose(1, 3) - last_keyframe.pose(1, 3), 2) +
                pow(curr_frame_.pose(2, 3) - last_keyframe.pose(2, 3), 2);
  if (dist > 2.0) {
    Frame keyframe;
    keyframe.cloud = cloud;
    keyframe.pose = curr_frame_.pose;
    AddKeyFrame(keyframe);
  }

  return curr_frame_;
}

void FrontEnd::AddKeyFrame(Frame &keyframe) {
  // ROS_INFO("AddKeyFrame++");
  // Update local keyframe and local map
  local_keyframe_.push_back(keyframe);
  if (local_keyframe_.size() > 20) {
    local_keyframe_.pop_front();
  }
  local_map_.reset();
  for (Frame &keyframe : local_keyframe_) {
    Cloud transformed_cloud;
    pcl::transformPointCloud(*(keyframe.cloud.cloud_ptr),
                             *(transformed_cloud.cloud_ptr), keyframe.pose);
    *(local_map_.cloud_ptr) += *(transformed_cloud.cloud_ptr);
  }

  // Setting point cloud to be aligned to.
  Cloud filtered_local_map;
  local_map_filter_ptr_->Filter(local_map_, filtered_local_map);
  // ROS_INFO_STREAM("local_map_: " << local_map_.cloud_ptr->size()
  //                                << ", filtered: "
  //                                << filtered_local_map.cloud_ptr->size());
  registration_ptr_->SetInputTarget(filtered_local_map);

  // Update global keyfram and global map
  // ROS_INFO("AddKeyFrame--");
}

void FrontEnd::SetInitPose(const Eigen::Matrix4f &init_pose) {
  ROS_INFO_STREAM("SetInitPose: " << init_pose);
  init_pose_ = init_pose;
  curr_frame_.pose = init_pose;
  last_pose_ = init_pose;
  predict_pose_ = init_pose;
}
}  // namespace slam_for_autonomous_vehicle