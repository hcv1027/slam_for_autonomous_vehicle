#include "models/registration/ndt_registration.h"
#include <string>

namespace slam_for_autonomous_vehicle {

NdtRegistration::NdtRegistration() {
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

void NdtRegistration::SetInputTarget(const Cloud &cloud) {
  ndt_.setInputTarget(cloud.cloud_ptr);
}

void NdtRegistration::ScanMatch(const Cloud &src_cloud,
                                const Eigen::Matrix4f &predict_pose,
                                Cloud &output_cloud,
                                Eigen::Matrix4f &output_pose) {
  ndt_.setInputSource(src_cloud.cloud_ptr);
  ndt_.align(*output_cloud.cloud_ptr, predict_pose);
  output_pose = ndt_.getFinalTransformation();
  // ROS_INFO_STREAM("ndt align: " << output_cloud.cloud_ptr->size());
}
}  // namespace slam_for_autonomous_vehicle