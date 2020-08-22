#include "models/voxel_filter/voxel_filter.h"

namespace slam_for_autonomous_vehicle {
VoxelFilter::VoxelFilter(float leaf_size) {
  filter_.setLeafSize(leaf_size, leaf_size, leaf_size);  // Unit: meter
}

void VoxelFilter::Filter(const Cloud &input_cloud, Cloud &output_cloud) {
  filter_.setInputCloud(input_cloud.cloud_ptr);
  filter_.filter(*output_cloud.cloud_ptr);
}
}  // namespace slam_for_autonomous_vehicle