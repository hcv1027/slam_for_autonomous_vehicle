#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include "data_subscriber/cloud_subscriber.h"
#include "data_subscriber/gnss_subscriber.h"
#include "data_subscriber/imu_subscriber.h"

using slam_for_autonomous_vehicle::CloudSubscriber;
using slam_for_autonomous_vehicle::GnssSubscriber;
using slam_for_autonomous_vehicle::ImuSubscriber;
using std::string;
// using Eigen;

bool TransformToMatrix(const tf::StampedTransform& transform,
                       Eigen::Matrix4f& transform_matrix) {
  Eigen::Translation3f tl_btol(transform.getOrigin().getX(),
                               transform.getOrigin().getY(),
                               transform.getOrigin().getZ());

  double roll, pitch, yaw;
  tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
  Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

  // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
  transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "slam_node");
  ros::NodeHandle nh;

  string cloud_topic;
  nh.setParam("cloud_topic", "/kitti/velo/pointcloud");
  nh.setParam("gnss_topic", "/kitti/oxts/gps/fix");
  // nh.setParam("gnss_topic", "/kitti/velo/pointcloud");
  nh.setParam("imu_topic", "/kitti/oxts/imu");

  string lidar_frame = "velo_link";
  string imu_frame = "imu_link";
  tf::TransformListener listener;
  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
  try {
    tf::StampedTransform transform;
    listener.waitForTransform(imu_frame, lidar_frame, ros::Time(0),
                              ros::Duration(1.0));
    listener.lookupTransform(imu_frame, lidar_frame, ros::Time(0), transform);
    TransformToMatrix(transform, transform_matrix);
    ROS_INFO_STREAM("transform_matrix:\n" << transform_matrix);
  } catch (tf::TransformException& ex) {
    // ROS_INFO_STREAM("No transform: " << ex);
  }

  // Eigen::Matrix3f m1;
  // m1 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  // Eigen::Matrix3f m2;
  // m2 << 1, 1, 1, 2, 2, 2, 3, 3, 3;
  // ROS_INFO_STREAM("m1:\n" << m1);
  // ROS_INFO_STREAM("m2:\n" << m2);
  // m1 *= m2;
  // ROS_INFO_STREAM("m1 *= m2:\n" << m1);

  CloudSubscriber cloud_sub(nh, 100);
  GnssSubscriber gnss_sub(nh, 100);
  ImuSubscriber imu_sub(nh, 100);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}