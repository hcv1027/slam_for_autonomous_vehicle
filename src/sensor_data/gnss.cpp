#include "sensor_data/gnss.h"

GeographicLib::LocalCartesian slam_for_autonomous_vehicle::Gnss::geo_converter;

namespace slam_for_autonomous_vehicle {
void Gnss::InitOrigin() { geo_converter.Reset(latitude, longitude, altitude); }

void Gnss::UpdateXYZ() {
  geo_converter.Forward(latitude, longitude, altitude, local_E, local_N,
                        local_U);
}

Gnss::Gnss(const sensor_msgs::NavSatFix& data) {
  time_stamp = data.header.stamp.toSec();
  longitude = data.longitude;
  latitude = data.latitude;
  altitude = data.altitude;
  status = data.status.status;
  service = data.status.service;
}

Gnss::Gnss(const Gnss& gnss) {
  time_stamp = gnss.time_stamp;
  longitude = gnss.longitude;
  latitude = gnss.latitude;
  altitude = gnss.altitude;
}
}  // namespace slam_for_autonomous_vehicle