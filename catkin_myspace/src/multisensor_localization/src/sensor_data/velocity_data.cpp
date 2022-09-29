#include "../../include/sensor_data/velocity_data.hpp"

namespace multisensor_localization
{

    /**
     * @brief  轮速计 时间同步
     * @note 线性插值
     * @todo
     **/
    bool VelocityData::SyncData(deque<VelocityData> &unsynced_data_buff,
                                deque<VelocityData> &synced_data_buff,
                                double sync_time)
    {
        /*排除异常状态*/
        while (unsynced_data_buff.size() >= 2)
        {
            LOG(INFO) << endl
                      << fontColorWhiteBold << ">>-->>-->>--debug point-->>-->>-->>" << endl
                      << fontColorYellow << "SyncData测试" << fontColorReset << endl
                      << fontColorBlue << "[0] " << unsynced_data_buff.at(0).time_stamp_ << endl
                      << fontColorBlue << "[1] " << unsynced_data_buff.at(1).time_stamp_ << endl
                      << fontColorBlue << "sync_time " << sync_time << endl
                      << fontColorBlue << "[1]-[0] " << unsynced_data_buff.at(1).time_stamp_ - unsynced_data_buff.at(0).time_stamp_ << endl
                      << fontColorWhiteBold << "<<--<<--<<--debug point--<<--<<--<<" << endl
                      << endl;
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
            /*上述4种情况均不存在 跳出此循环*/
            break;
        }

        /*异常5:仅存在[0]数据*/
        if (unsynced_data_buff.size() < 2)
            return false;

        /*a(1-t)+b插值系数计算*/
        VelocityData front_data = unsynced_data_buff.at(0);
        VelocityData back_data = unsynced_data_buff.at(1);
        double front_scale = (back_data.time_stamp_ - sync_time) /
                             (back_data.time_stamp_ - front_data.time_stamp_);
        double back_scale = (sync_time - front_data.time_stamp_) /
                            (back_data.time_stamp_ - front_data.time_stamp_);

        VelocityData synced_data;

        /*不插值量拷贝*/
        synced_data.time_stamp_ = sync_time;
        /*线速度插值*/
        synced_data.linear_velocity_.x =
            front_data.linear_velocity_.x * front_scale +
            back_data.linear_velocity_.x * back_scale;

        synced_data.linear_velocity_.y =
            front_data.linear_velocity_.y * front_scale +
            back_data.linear_velocity_.y * back_scale;

        synced_data.linear_velocity_.z =
            front_data.linear_velocity_.z * front_scale +
            back_data.linear_velocity_.z * back_scale;

        /*角速度插值*/
        synced_data.angular_velocity_.x =
            front_data.angular_velocity_.x * front_scale +
            back_data.angular_velocity_.x * back_scale;

        synced_data.angular_velocity_.y =
            front_data.angular_velocity_.y * front_scale +
            back_data.angular_velocity_.y * back_scale;

        synced_data.angular_velocity_.z =
            front_data.angular_velocity_.z * front_scale +
            back_data.angular_velocity_.z * back_scale;

        synced_data_buff.push_back(synced_data);

        return true;
    }

} // namespace multisensor_localization