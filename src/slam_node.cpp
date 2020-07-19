#include <ros/ros.h>
#include "data_subscriber/cloud_subscriber.h"
#include "data_subscriber/gnss_subscriber.h"

using slam_for_autonomous_vehicle::CloudSubscriber;
using slam_for_autonomous_vehicle::GnssSubscriber;

int main(int argc, char *argv[]) {
  // google::InitGoogleLogging(argv[0]);
  // FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  // FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "slam_node");
  ros::NodeHandle nh;

  std::string cloud_topic;
  nh.setParam("cloud_topic", "/kitti/velo/pointcloud");
  nh.setParam("gnss_topic", "/kitti/oxts/gps/fix");
  // nh.setParam("gnss_topic", "/kitti/velo/pointcloud");
  nh.setParam("imu_topic", "/kitti/oxts/imu");

  CloudSubscriber cloud_sub(nh, 100);
  GnssSubscriber gnss_sub(nh, 100);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}