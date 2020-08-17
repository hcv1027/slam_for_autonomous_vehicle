#include "front_end.h"
#include <pcl/common/transforms.h>
#include <ros/console.h>
#include <cmath>

using std::pow;
using std::sqrt;
using std::string;

namespace slam_for_autonomous_vehicle {
FrontEnd::FrontEnd(ros::NodeHandle &nh) : nh_(nh) {
  float leaf_size = nh_.param(string("leaf_size"), 1.2);
  cloud_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);  // Unit: meter

  // curr_pose_ = Eigen::Matrix4f::Identity();
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
  ndt_.setMaximumIterations(30);
}

FrontEnd::Frame FrontEnd::Update(const Cloud &cloud) {
  std::vector<int> indices;
  Cloud no_nan_cloud;
  Cloud filteded_cloud;
  pcl::removeNaNFromPointCloud(*(cloud.cloud_ptr), *(no_nan_cloud.cloud_ptr),
                               indices);
  cloud_filter_.setInputCloud(no_nan_cloud.cloud_ptr);
  cloud_filter_.filter(*(filteded_cloud.cloud_ptr));
  ROS_INFO_STREAM("Filter input cloud: " << filteded_cloud.cloud_ptr->size());

  // curr_frame_.cloud = filteded_cloud;
  // curr_frame_.pose = Eigen::Matrix4f::Identity();
  // return curr_frame_;

  // Add first key frame
  if (local_keyframe_.size() == 0) {
    /* Frame keyframe;
    keyframe.cloud = cloud;
    keyframe.pose = Eigen::Matrix4f::Identity(); */
    curr_frame_.cloud = no_nan_cloud;
    curr_frame_.pose = Eigen::Matrix4f::Identity();
    AddKeyFrame(curr_frame_);
    return curr_frame_;
  }

  // Setting point cloud to be aligned.
  Cloud output_cloud;
  ROS_INFO("Start ndt");
  ndt_.setInputSource(filteded_cloud.cloud_ptr);
  ROS_INFO_STREAM("ndt align: " << output_cloud.cloud_ptr->size());
  ndt_.align(*output_cloud.cloud_ptr, predict_pose_);
  ROS_INFO("ndt get final");
  auto temp_pose = ndt_.getFinalTransformation();
  ROS_INFO_STREAM("temp_pose: " << temp_pose);
  curr_frame_.pose = temp_pose;
  // curr_frame_.pose = ndt_.getFinalTransformation();
  ROS_INFO("After ndt");

  // Prdtict next pose formula: next_pose = curr_pose * step_pose
  Eigen::Matrix4f step_pose = last_pose_.inverse() * curr_frame_.pose;
  predict_pose_ = curr_frame_.pose * step_pose;
  last_pose_ = curr_frame_.pose;
  ROS_INFO("After predict");

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
  ROS_INFO("AddKeyFrame++");
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
  cloud_filter_.setInputCloud(local_map_.cloud_ptr);
  cloud_filter_.filter(*filtered_local_map.cloud_ptr);
  ROS_INFO_STREAM("local_map_: " << local_map_.cloud_ptr->size()
                                 << ", filtered: "
                                 << filtered_local_map.cloud_ptr->size());
  ndt_.setInputTarget(filtered_local_map.cloud_ptr);

  ROS_INFO("Test ndt");
  // Cloud output_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  ndt_.setInputSource(filtered_local_map.cloud_ptr);
  ndt_.align(*output_cloud, predict_pose_);
  ROS_INFO("Test ndt get final");

  // Update global keyfram and global map
  ROS_INFO("AddKeyFrame--");
}

void FrontEnd::SetInitPose(const Eigen::Matrix4f &init_pose) {
  init_pose_ = init_pose;
  curr_frame_.pose = init_pose;
  last_pose_ = init_pose;
  predict_pose_ = init_pose;
}
}  // namespace slam_for_autonomous_vehicle