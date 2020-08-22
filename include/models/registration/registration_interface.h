#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_REGISTRATION_REGISTRATION_INTERFACE_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_REGISTRATION_REGISTRATION_INTERFACE_H_

#include <Eigen/Dense>
#include "sensor_data/cloud.h"

namespace slam_for_autonomous_vehicle {

class RegistrationInterface {
 public:
  RegistrationInterface() = default;
  virtual ~RegistrationInterface() = default;

  virtual void SetInputTarget(const Cloud &cloud) = 0;

  virtual void ScanMatch(const Cloud &src_cloud,
                         const Eigen::Matrix4f &predict_pose,
                         Cloud &output_cloud, Eigen::Matrix4f &output_pose) = 0;
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_REGISTRATION_REGISTRATION_INTERFACE_H_