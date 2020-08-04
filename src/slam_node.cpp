#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <deque>
#include "data_subscriber/cloud_subscriber.h"
#include "data_subscriber/gnss_subscriber.h"
#include "data_subscriber/imu_subscriber.h"
#include "tf_listener.h"

using slam_for_autonomous_vehicle::CloudSubscriber;
using slam_for_autonomous_vehicle::GnssSubscriber;
using slam_for_autonomous_vehicle::ImuSubscriber;
using slam_for_autonomous_vehicle::Imu;
using slam_for_autonomous_vehicle::Gnss;
using slam_for_autonomous_vehicle::Cloud;
using slam_for_autonomous_vehicle::TfListener;
using std::string;
using std::deque;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "slam_node");
  ros::NodeHandle nh;

  string cloud_topic;
  nh.setParam("cloud_topic", "/kitti/velo/pointcloud");
  nh.setParam("gnss_topic", "/kitti/oxts/gps/fix");
  // nh.setParam("gnss_topic", "/kitti/velo/pointcloud");
  nh.setParam("imu_topic", "/kitti/oxts/imu");

  string lidar_frame = "velo_link";
  string imu_frame = "imu_link";
  TfListener tf_listener;
  Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
  int try_count = 0;
  bool transform_result = false;
  do {
    try_count++;
    transform_result =
        tf_listener.GetTransformMatrix(imu_frame, lidar_frame, lidar_to_imu);
  } while (!transform_result && try_count < 10);
  if (!transform_result) {
    ROS_ERROR_STREAM("Fail to get transform from " << lidar_frame << " to "
                                                   << imu_frame);
    exit(0);
  }

  CloudSubscriber cloud_sub(nh, 100);
  GnssSubscriber gnss_sub(nh, 100);
  ImuSubscriber imu_sub(nh, 100);
  deque<Cloud> cloud_data;
  deque<Imu> imu_data;
  deque<Gnss> gnss_data;
  bool init_odometry = true;

  ros::Rate rate(100);
  while (ros::ok()) {
    cloud_sub.get_data(cloud_data);
    imu_sub.get_data(imu_data);
    gnss_sub.get_data(gnss_data);
    if (cloud_data.size()) {
    }

    Cloud &cloud = cloud_data.front();
    Imu &imu = imu_data.front();
    Gnss &gnss = gnss_data.front();

    if (init_odometry) {
      init_odometry = false;
    }
    // Cloud cloud =
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}