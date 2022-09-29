#include "../../include/sensor_data/gnss_data.hpp"

namespace multisensor_localization
{
    GeographicLib::LocalCartesian GnssData::geo_converter_; /*静态变量类内定义类外初始化*/
    bool GnssData::origin_position_inited_ = false;

    /**
     * @brief 设置东北天坐标系初始化位置
     * @note
     * @todo
     **/
    void GnssData::InitOriginPosition()
    {
        geo_converter_.Reset(latitude_, longtitude_, altitude_);
        origin_position_inited_ = true;
    }

    /**
     * @brief 更新东北天坐标系XYZ数据
     * @note 必须初始化后才可以更新
     * @todo
     **/
    void GnssData::UpdateXYZ()
    {
        if (origin_position_inited_ == false)
        {
            // cout<<"东北天坐标系未初始化"<<endl;
        }
        geo_converter_.Forward(latitude_, longtitude_, altitude_, local_E_, local_N_, local_U_);
    }

    /**
     * @brief 传感器数据时间同步
     * @note
     * @todo
     **/
    bool GnssData::SyncData(deque<GnssData> &unsynced_data_buff,
                            deque<GnssData> &synced_data_buff,
                             double sync_time)
    {
        /*排除异常状态*/
        while (unsynced_data_buff.size() >= 2)
        {
            /*异常1:sync_time-->[0]-->[1]，无法同步*/
            if (unsynced_data_buff.at(0).time_stamp_ > sync_time)
                return false;
            /*异常2:sync_time,[0]数据丢弃后跳过后续判断*/
            if (unsynced_data_buff.at(1).time_stamp_ < sync_time)
            {
                unsynced_data_buff.pop_front();
                continue;
            }
            /*异常3:[0]--(时间差过大)-->sync_time-->[1]  数据丢弃*/
            if (sync_time - unsynced_data_buff.at(0).time_stamp_ > 0.2)
            {
                unsynced_data_buff.pop_front();
                break;
            }
            /*异常4:[0]-->sync_time--(时间差过大)-->[1] 数据丢弃*/
            if (unsynced_data_buff.at(1).time_stamp_ - sync_time > 0.2)
            {
                unsynced_data_buff.pop_front();
                break;
            }
            /*上述4种情况均不存在，则可以正常插值，跳出此循环*/
            break;
        }

        if (unsynced_data_buff.size() < 2)
            return false;

        /*a(1-t)+b插值系数计算*/

        GnssData front_data = unsynced_data_buff.at(0);
        GnssData back_data = unsynced_data_buff.at(1);
        double front_scale = (back_data.time_stamp_ - sync_time) / (back_data.time_stamp_ - front_data.time_stamp_);
        double back_scale = (sync_time - front_data.time_stamp_) / (back_data.time_stamp_ - front_data.time_stamp_);

        GnssData synced_data;
        /*不插值量拷贝*/
        synced_data.time_stamp_ = sync_time;
        synced_data.status_ = back_data.time_stamp_;
        /*经度、纬度、海拔插值*/
        synced_data.longtitude_ =
            front_data.longtitude_ * front_scale +
            back_data.longtitude_ * back_scale;

        synced_data.latitude_ =
            front_data.latitude_ * front_scale +
            back_data.latitude_ * back_scale;

        synced_data.altitude_ =
            front_data.altitude_ * front_scale +
            back_data.altitude_ * back_scale;
        /*东北天坐标系插值*/
        synced_data.local_E_ =
            front_data.local_E_ * front_scale +
            back_data.local_E_ * back_scale;

        synced_data.local_N_ =
            front_data.local_N_ * front_scale +
            back_data.local_N_ * back_scale;

        synced_data.local_U_ =
            front_data.local_U_ * front_scale +
            back_data.local_U_ * back_scale;

        synced_data_buff.push_back(synced_data);

        return true;
    }
}
