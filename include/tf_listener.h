#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_TF_LISTENER_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_TF_LISTENER_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <string>

namespace slam_for_autonomous_vehicle {

class TfListener {
 public:
  TfListener() = default;
  ~TfListener() = default;

  bool GetTransformMatrix(const std::string &base_frame,
                          const std::string &child_frame,
                          Eigen::Matrix4f &transform_matrix);

 private:
  void TransformToMatrix(const tf::StampedTransform &transform,
                         Eigen::Matrix4f &transform_matrix);

 private:
  tf::TransformListener listener_;
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_TF_LISTENER_H_