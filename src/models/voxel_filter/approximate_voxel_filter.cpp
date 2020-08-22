#include "models/voxel_filter/approximate_voxel_filter.h"

namespace slam_for_autonomous_vehicle {
ApproximateVoxelFilter::ApproximateVoxelFilter(float leaf_size) {
  filter_.setLeafSize(leaf_size, leaf_size, leaf_size);  // Unit: meter
}

void ApproximateVoxelFilter::Filter(const Cloud &input_cloud,
                                    Cloud &output_cloud) {
  filter_.setInputCloud(input_cloud.cloud_ptr);
  filter_.filter(*output_cloud.cloud_ptr);
}
}  // namespace slam_for_autonomous_vehicle