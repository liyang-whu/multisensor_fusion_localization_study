/*
 * @Description:自定义点云数据结构
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#include "../../include/sensor_data/gnss_data.hpp"


namespace multisensor_localization
{
    /*类内静态变量初始化*/
    bool GnssData::origin_position_inited_ = false;
    GeographicLib::LocalCartesian GnssData::geo_converter_;

    /**
     * @brief 初始化东北天坐标系原点
     * @note
     * @todo
     **/
    void GnssData::InitOriginPosition()
    {
        geo_converter_.Reset(latitude_, longitude_, altitude_);
        origin_position_inited_ = true;
    }

    /**
     * @brief 更新东北天三轴位移
     * @note
     * @todo
     **/
    void GnssData::UpdateXYZ()
    {
        if (!origin_position_inited_)
        {
            debug_tools::DebugTools::Debug_Error("东北天坐标系未初始化");
        }
        geo_converter_.Forward(latitude_, longitude_, altitude_,
                               local_E_, local_N_, local_U_);
    }

    /**
     * @brief gnss时间同步
     * @note
     * @todo
     **/
    bool GnssData::SyncData(std::deque<GnssData> &unsynced_data_buff,
                            std::deque<GnssData> &synced_data_buff, double sync_time)
    {
        while (unsynced_data_buff.size() >= 2)
        {
            /*异常1:sync_time>[0]>[1]*/
            if (unsynced_data_buff.at(0).time_stamp_ > sync_time)
                return false;
            /*异常2:[0]>[1]>sync_time*/
            if (unsynced_data_buff.at(1).time_stamp_ < sync_time)
            {
                unsynced_data_buff.pop_front();
                continue;
            }
            /*异常3:[0]>>sync_time>[1]*/
            if (sync_time - unsynced_data_buff.at(0).time_stamp_ > 0.2)
            {
                unsynced_data_buff.pop_front();
                break;
            }
            /*异常4:[0]>sync_time>>[2]*/
            if (unsynced_data_buff.at(1).time_stamp_ - sync_time > 0.2)
            {
                unsynced_data_buff.pop_front();
                break;
            }
            break;
        }

        if (unsynced_data_buff.size() < 2)
            return false;

        /*线性插值a(1-t)+bt系数计算*/
        GnssData front_data = unsynced_data_buff.at(0);
        GnssData back_data = unsynced_data_buff.at(1);
        double front_scale = (back_data.time_stamp_ - sync_time) / (back_data.time_stamp_ - front_data.time_stamp_);
        double back_scale = (sync_time - front_data.time_stamp_) / (back_data.time_stamp_ - front_data.time_stamp_);

        GnssData synced_data;

        /*非同步量拷贝*/
        synced_data.time_stamp_ = sync_time;
        synced_data.status_ = back_data.status_;
        /*同步量拷贝*/
        synced_data.longitude_ = front_data.longitude_ * front_scale + back_data.longitude_ * back_scale;
        synced_data.latitude_ = front_data.latitude_ * front_scale + back_data.latitude_ * back_scale;
        synced_data.altitude_ = front_data.altitude_ * front_scale + back_data.altitude_ * back_scale;

        synced_data.local_E_ = front_data.local_E_ * front_scale + back_data.local_E_ * back_scale;
        synced_data.local_N_ = front_data.local_N_ * front_scale + back_data.local_N_ * back_scale;
        synced_data.local_U_ = front_data.local_U_ * front_scale + back_data.local_U_ * back_scale;

        /*压入队列引用传参*/
        synced_data_buff.push_back(synced_data);

        return true;
    }
} // multisensor_localization