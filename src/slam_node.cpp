#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <deque>
#include "data_subscriber/cloud_subscriber.h"
#include "data_subscriber/gnss_subscriber.h"
#include "data_subscriber/imu_subscriber.h"
#include "front_end.h"
#include "publisher/odometry_publisher.h"
#include "tf_listener.h"

using slam_for_autonomous_vehicle::CloudSubscriber;
using slam_for_autonomous_vehicle::GnssSubscriber;
using slam_for_autonomous_vehicle::ImuSubscriber;
using slam_for_autonomous_vehicle::Imu;
using slam_for_autonomous_vehicle::Gnss;
using slam_for_autonomous_vehicle::Cloud;
using slam_for_autonomous_vehicle::TfListener;
using slam_for_autonomous_vehicle::FrontEnd;
using slam_for_autonomous_vehicle::OdometryPublisher;
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

  string map_frame = "map";
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

  OdometryPublisher odom_pub(nh, "lidar_odom", map_frame, lidar_frame);
  CloudSubscriber cloud_sub(nh, 100);
  GnssSubscriber gnss_sub(nh, 100);
  ImuSubscriber imu_sub(nh, 100);
  deque<Cloud> cloud_data_buff;
  deque<Imu> imu_data_buff;
  deque<Gnss> gnss_data_buff;
  bool init_odometry = true;
  bool init_gnss = true;
  FrontEnd front_end(nh);

  ros::Rate rate(100);
  while (ros::ok()) {
    cloud_sub.get_data(cloud_data_buff);
    imu_sub.get_data(imu_data_buff);
    gnss_sub.get_data(gnss_data_buff);
    if (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 &&
        gnss_data_buff.size() > 0) {
      Cloud &cloud = cloud_data_buff.front();
      Imu &imu = imu_data_buff.front();
      Gnss &gnss = gnss_data_buff.front();
      double dt = cloud.time_stamp - imu.time_stamp;
      if (dt < -0.05) {
        cloud_data_buff.pop_front();
      } else if (dt > 0.05) {
        imu_data_buff.pop_front();
        gnss_data_buff.pop_front();
      } else {
        // Initialize GeographicLib::LocalCartesian
        if (!init_gnss) {
          init_gnss = false;
          gnss.InitOrigin();
        }
        gnss.UpdateXYZ();

        Eigen::Matrix4f odometry = Eigen::Matrix4f::Identity();
        odometry(0, 3) = gnss.local_E;
        odometry(1, 3) = gnss.local_N;
        odometry(2, 3) = gnss.local_U;
        odometry.block<3, 3>(0, 0) = imu.GetOrientationMatrix();
        odometry *= lidar_to_imu;
        if (init_odometry) {
          init_odometry = false;
          front_end.SetInitPose(odometry);
        }
        FrontEnd::Frame curr_frame = front_end.Update(cloud);

        // Publish result
        odom_pub.publish(curr_frame.pose);

        cloud_data_buff.pop_front();
        imu_data_buff.pop_front();
        gnss_data_buff.pop_front();
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}