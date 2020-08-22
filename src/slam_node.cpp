#include <nav_msgs/Odometry.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <deque>
#include <iostream>
#include "data_subscriber/cloud_subscriber.h"
#include "data_subscriber/gnss_subscriber.h"
#include "data_subscriber/imu_subscriber.h"
#include "front_end.h"
#include "publisher/cloud_publisher.h"
#include "publisher/odometry_publisher.h"
#include "tf_listener.h"

using slam_for_autonomous_vehicle::Cloud;
using slam_for_autonomous_vehicle::CloudPublisher;
using slam_for_autonomous_vehicle::CloudSubscriber;
using slam_for_autonomous_vehicle::FrontEnd;
using slam_for_autonomous_vehicle::Gnss;
using slam_for_autonomous_vehicle::GnssSubscriber;
using slam_for_autonomous_vehicle::Imu;
using slam_for_autonomous_vehicle::ImuSubscriber;
using slam_for_autonomous_vehicle::OdometryPublisher;
using slam_for_autonomous_vehicle::TfListener;
using std::deque;
using std::string;

// apt install libvtk6.3-qt libvtk6.3 libvtk6-dev  ros-noetic-pcl-ros

int test_ndt() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "/root/code/ros_ws/my_project/src/slam_for_autonomous_vehicle/data/"
          "room_scan1.pcd",
          *target_cloud) == -1) {
    PCL_ERROR("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size()
            << " data points from room_scan1.pcd" << std::endl;

  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "/root/code/ros_ws/my_project/src/slam_for_autonomous_vehicle/data/"
          "room_scan2.pcd",
          *input_cloud) == -1) {
    PCL_ERROR("Couldn't read file room_scan2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size()
            << " data points from room_scan2.pcd" << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of
  // registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud(input_cloud);
  approximate_voxel_filter.filter(*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size()
            << " data points from room_scan2.pcd" << std::endl;

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon(0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize(0.1);
  // Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution(1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations(35);

  // Setting point cloud to be aligned.
  ndt.setInputSource(filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget(target_cloud);

  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

  // Calculating required rigid transform to align the input cloud to the target
  // cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align(*output_cloud, init_guess);

  std::cout << "Normal Distributions Transform has converged:"
            << ndt.hasConverged() << " score: " << ndt.getFitnessScore()
            << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud(*input_cloud, *output_cloud,
                           ndt.getFinalTransformation());

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII("room_scan2_transformed.pcd", *output_cloud);

  // Initializing point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer_final(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer_final->setBackgroundColor(0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(
      target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color,
                                             "target cloud");
  viewer_final->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(
      output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color,
                                             "output cloud");
  viewer_final->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem(1.0, "global");
  viewer_final->initCameraParameters();

  // Wait until visualizer window is closed.
  ros::Rate rate(100);
  while (!viewer_final->wasStopped()) {
    viewer_final->spinOnce(100);
    rate.sleep();
  }

  return (0);
}

int main(int argc, char *argv[]) {
  std::cout << PCL_VERSION << std::endl;
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
  CloudPublisher cloud_pub(nh, "lidar_cloud", lidar_frame);
  CloudSubscriber cloud_sub(nh, 100);
  GnssSubscriber gnss_sub(nh, 100);
  ImuSubscriber imu_sub(nh, 100);
  deque<Cloud> cloud_data_buff;
  deque<Imu> imu_data_buff;
  deque<Gnss> gnss_data_buff;
  bool init_odometry = true;
  bool init_gnss = true;
  FrontEnd front_end(nh);

  // test_ndt();

  ros::Rate rate(100);
  while (ros::ok()) {
    // ROS_INFO("loop start");
    cloud_sub.get_data(cloud_data_buff);
    // ROS_INFO("cloud_data_buff size: %d", cloud_data_buff.size());
    imu_sub.get_data(imu_data_buff);
    // ROS_INFO("imu_data_buff size: %d", imu_data_buff.size());
    gnss_sub.get_data(gnss_data_buff);
    // ROS_INFO("gnss_data_buff size: %d", gnss_data_buff.size());
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
        if (init_gnss) {
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
        // ROS_INFO_STREAM("odometry: " << odometry);
        FrontEnd::Frame curr_frame = front_end.Update(cloud);
        // ROS_INFO("Update complete");

        // Publish result
        odom_pub.publish(curr_frame.pose);
        cloud_pub.publish(curr_frame.cloud);

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