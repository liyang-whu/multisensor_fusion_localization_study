/*
 * @Description: 自定义的gnss数据类型
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */

#ifndef GNSS_DATA_HPP_
#define GNSS_DATA_HPP_

#include <deque>
#include <GeographicLib/LocalCartesian.hpp>

namespace multisensor_localization
{
    class GnssData
    {
    public:
        double time_stamp_ = 0.0;
        double longtitude_ = 0.0, latitude_ = 0.0, altitude_ = 0.0;
        double local_E_ = 0.0, local_N_ = 0.0, local_U_ = 0.0;
        int status_;
        int service_ = 0;

    private:
        static GeographicLib::LocalCartesian geo_converter_;
        static bool origin_position_inited_;

    public:
        void InitOriginPosition();
        void UpdateXYZ();
        static bool SyncData(std::deque<GnssData> &unsynced_data_buff,
                             std::deque<GnssData> &synced_data_buff,
                             const double sync_time);
    };
}

#endif