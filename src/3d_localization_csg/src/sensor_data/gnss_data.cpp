#include "sensor_data/gnss_data.hpp"

#include <iostream>

#include "glog/logging.h"
// static member must init outside of class
bool h_x::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian h_x::GNSSData::geo_converter;

namespace h_x {

void GNSSData::InitOriginPosition(double& latitude, double& longitude, double& altitude) {
  // geo_converter.Reset(latitude, longitude, altitude);
    geo_converter.Reset(latitude, longitude, altitude);
  // geo_converter.Reset(48.9825914868, 8.39040283786, 116.388149261);
  // geo_converter.Reset(31.1506836219693000, 121.1702651870648282, 4.3672881126403809);  // 移动智地 高精度车数据
  // geo_converter.Reset(30.91082689688523, 118.3012412538284, 17.67815780639648); // 金色水岸
  // geo_converter.Reset(30.913001017445331797, 118.30027226487733572, 17.135038375854492188); // 滨玉线
  origin_position_inited = true;
}

void GNSSData::UpdateXYZ() {
  if (!origin_position_inited) {
    // LOG(WARNING) << "GeoConverter has not set origin position";
    std::cout << "GeoConverter has not set origin position";
  }
  geo_converter.Forward(latitude, longitude, altitude, local_E, local_N,
                        local_U);
}

bool GNSSData::SyncData(std::deque<GNSSData, Eigen::aligned_allocator<GNSSData>>& UnsyncedData,
                        std::deque<GNSSData, Eigen::aligned_allocator<GNSSData>>& SyncedData,
                        const double& sync_time) {
  while (UnsyncedData.size() >= 2) {
    if (UnsyncedData.front().time > sync_time) {
      return false;
    }

    if (UnsyncedData.at(1).time < sync_time) {
      UnsyncedData.pop_front();
      continue;
    }
    if (sync_time - UnsyncedData.front().time > 2) {
      UnsyncedData.pop_front();
      return false;
    }
    if (UnsyncedData.at(1).time - sync_time > 2) {
      UnsyncedData.pop_front();
      return false;
    }
    break;
  }
    if (UnsyncedData.size() < 2) {
        // LOG(ERROR) << " UnsyncedData->size less than 2 " << std::endl;
        if(UnsyncedData.size()) {
            SyncedData.emplace_back(UnsyncedData.at(0));
            return true;
        }
        return false;
    }
  GNSSData front_data = UnsyncedData.at(0);
  GNSSData back_data = UnsyncedData.at(1);
  GNSSData synced_data;

  double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
  synced_data.time = sync_time;
  synced_data.status = back_data.status;
  synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
  synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
  synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
  synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
  synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
  synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

  SyncedData.emplace_back(synced_data);
  
  return true;
}

}  // namespace h_x