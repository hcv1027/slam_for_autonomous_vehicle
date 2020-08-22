#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_FRONT_END_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_FRONT_END_H_

#include <Eigen/Dense>
#include <deque>
#include <memory>
#include "models/registration/registration_interface.h"
#include "models/voxel_filter/voxel_filter_interface.h"
#include "sensor_data/cloud.h"

namespace slam_for_autonomous_vehicle {

class FrontEnd {
 public:
  struct Frame {
    Eigen::Matrix4f pose;
    Cloud cloud;
  };

  FrontEnd(ros::NodeHandle &nh);
  FrontEnd() = default;
  ~FrontEnd() = default;

  FrontEnd::Frame Update(const Cloud &cloud);

  void SetInitPose(const Eigen::Matrix4f &init_pose);

 private:
  void AddKeyFrame(Frame &keyframe);

 private:
  ros::NodeHandle nh_;
  std::shared_ptr<VoxelFilterInterface> cloud_filter_ptr_;
  std::shared_ptr<VoxelFilterInterface> local_map_filter_ptr_;
  std::shared_ptr<RegistrationInterface> registration_ptr_;

  Eigen::Matrix4f init_pose_;
  Eigen::Matrix4f last_pose_;
  Eigen::Matrix4f predict_pose_;
  Frame curr_frame_;

  std::deque<Frame> local_keyframe_;
  Cloud local_map_;
  std::deque<Frame> global_keyframe_;
  Cloud global_map_;
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_FRONT_END_H_