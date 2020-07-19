#ifndef SLAM_FOR_AUTONOMOUS_VEHICLE_SENSOR_DATA_GNSS_H_
#define SLAM_FOR_AUTONOMOUS_VEHICLE_SENSOR_DATA_GNSS_H_

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "Geocentric/LocalCartesian.hpp"

namespace slam_for_autonomous_vehicle {

struct Gnss {
  double time_stamp = 0.0;
  double longitude = 0.0;
  double latitude = 0.0;
  double altitude = 0.0;
  double local_E = 0.0;
  double local_N = 0.0;
  double local_U = 0.0;
  int status = 0;
  int service = 0;

 private:
  static GeographicLib::LocalCartesian geo_converter;

 public:
  void InitOrigin();
  void UpdateXYZ();

  Gnss() = default;
  Gnss(const sensor_msgs::NavSatFix& data);
  Gnss(const Gnss& gnss);
  ~Gnss() = default;
};

}  // namespace slam_for_autonomous_vehicle

#endif  // SLAM_FOR_AUTONOMOUS_VEHICLE_SENSOR_DATA_GNSS_H_