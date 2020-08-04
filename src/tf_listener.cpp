#include "tf_listener.h"

namespace slam_for_autonomous_vehicle {

bool TfListener::GetTransformMatrix(const std::string &base_frame,
                                    const std::string &child_frame,
                                    Eigen::Matrix4f &transform_matrix) {
  try {
    tf::StampedTransform transform;
    listener_.waitForTransform(base_frame, child_frame, ros::Time(0),
                               ros::Duration(1.0));
    listener_.lookupTransform(base_frame, child_frame, ros::Time(0), transform);
    TransformToMatrix(transform, transform_matrix);
    return true;
    // ROS_INFO_STREAM("transform_matrix:\n" << transform_matrix);
  } catch (tf::TransformException &ex) {
    ROS_WARN_STREAM("lookupTransform fail, base_frame: "
                    << base_frame << ", child_frame: " << child_frame);
  }
  return false;
}

void TfListener::TransformToMatrix(const tf::StampedTransform &transform,
                                   Eigen::Matrix4f &transform_matrix) {
  Eigen::Translation3f tl_btol(transform.getOrigin().getX(),
                               transform.getOrigin().getY(),
                               transform.getOrigin().getZ());
  double roll, pitch, yaw;
  tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
  Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

  // Transform matrix from child_frame to base_frame
  transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
}
}