/*
 * @Description:
 * @Author: Robotic Gang
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#ifndef GNSS_DATA_HPP_
#define GNSS_DATA_HPP_

#include <deque>

#include "GeographicLib/LocalCartesian.hpp"

namespace multisensor_localization
{
  class GnssData
  {
  public:
    double time_stamp_ = 0.0;
    double longitude_ = 0.0, latitude_ = 0.0, altitude_ = 0.0;
    double local_E_ = 0.0, local_N_ = 0.0, local_U_ = 0.0;
    int status_ = 0;
    int service_ = 0;

  private:
    static GeographicLib::LocalCartesian geo_converter_;
    static bool origin_position_inited_;

  public:
    void InitOriginPosition();
    void UpdateXYZ();
    static bool SyncData(std::deque<GnssData> &unsynced_data_buff,
                         std::deque<GnssData> &synced_data_buff,
                         double sync_time);
  };

} // namespace multisensor_localization

#endif