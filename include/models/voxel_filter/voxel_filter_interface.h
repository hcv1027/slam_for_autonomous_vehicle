#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_VOXEL_FILTER_VOXEL_FILTER_INTERFACE_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_VOXEL_FILTER_VOXEL_FILTER_INTERFACE_H_

#include "sensor_data/cloud.h"

namespace slam_for_autonomous_vehicle {

class VoxelFilterInterface {
 public:
  VoxelFilterInterface() = default;
  virtual ~VoxelFilterInterface() = default;

  virtual void Filter(const Cloud &input_cloud, Cloud &output_cloud) = 0;
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_VOXEL_FILTER_VOXEL_FILTER_INTERFACE_H_