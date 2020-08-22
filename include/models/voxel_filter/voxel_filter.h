#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_VOXEL_FILTER_VOXEL_FILTER_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_VOXEL_FILTER_VOXEL_FILTER_H_

#include <pcl/filters/voxel_grid.h>
#include "models/voxel_filter/voxel_filter_interface.h"
#include "sensor_data/cloud.h"

namespace slam_for_autonomous_vehicle {

class VoxelFilter : public VoxelFilterInterface {
 public:
  VoxelFilter(float leaf_size);
  virtual ~VoxelFilter() = default;

  virtual void Filter(const Cloud &input_cloud, Cloud &output_cloud);

 private:
  // User must provides leaf_size
  VoxelFilter() = default;

 private:
  pcl::VoxelGrid<pcl::PointXYZ> filter_;
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_VOXEL_FILTER_VOXEL_FILTER_H_