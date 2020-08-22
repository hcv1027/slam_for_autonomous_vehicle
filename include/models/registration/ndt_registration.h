#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_REGISTRATION_NDT_REGISTRATION_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_REGISTRATION_NDT_REGISTRATION_H_

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Dense>
#include "models/registration/registration_interface.h"
#include "sensor_data/cloud.h"

namespace slam_for_autonomous_vehicle {

class NdtRegistration : public RegistrationInterface {
 public:
  NdtRegistration();
  virtual ~NdtRegistration() = default;

  virtual void SetInputTarget(const Cloud &cloud);

  virtual void ScanMatch(const Cloud &src_cloud,
                         const Eigen::Matrix4f &predict_pose,
                         Cloud &output_cloud, Eigen::Matrix4f &output_pose);

 private:
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_MODELS_REGISTRATION_NDT_REGISTRATION_H_