#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_VOXEL_FILTER_APPROXIMATE_VOXEL_FILTER_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_VOXEL_FILTER_APPROXIMATE_VOXEL_FILTER_H_

#include <pcl/filters/approximate_voxel_grid.h>
#include "models/voxel_filter/voxel_filter_interface.h"
#include "sensor_data/cloud.h"

namespace slam_for_autonomous_vehicle {

class ApproximateVoxelFilter : public VoxelFilterInterface {
 public:
  ApproximateVoxelFilter(float leaf_size);
  virtual ~ApproximateVoxelFilter() = default;

  virtual void Filter(const Cloud &input_cloud, Cloud &output_cloud);

 private:
  // User must provides leaf_size
  ApproximateVoxelFilter() = default;

 private:
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> filter_;
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_VOXEL_FILTER_APPROXIMATE_VOXEL_FILTER_H_