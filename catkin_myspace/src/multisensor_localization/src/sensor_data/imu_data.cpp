#include "../../include/sensor_data/imu_data.hpp"

namespace multisensor_localization
{
    /**
     * @brief 四元数转换为旋转矩阵
     * @note
     * @todo
     **/
    Eigen::Matrix3f ImuData::OrientationToRotation()
    {
        Eigen::Quaterniond q(orientation_.w, orientation_.x, orientation_.y, orientation_.z);
        Eigen::Matrix3f rotation = q.matrix().cast<float>();
        return rotation;
    }

    /**
     * @brief 传感器数据时间同步
     * @note
     * @todo
     **/
    bool ImuData::SyncData(std::deque<ImuData> &unsynced_data_buff,
                           std::deque<ImuData> &synced_data_buff,
                           const double sync_time)
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
            /*上述4种情况均不存在 跳出此循环*/
            break;
        }

        /*异常5:仅存在[0]数据*/
        if (unsynced_data_buff.size() < 2)
            return false;

        /*a(1-t)+b插值系数计算*/

        ImuData front_data = unsynced_data_buff.at(0);
        ImuData back_data = unsynced_data_buff.at(1);
        double front_scale = (back_data.time_stamp_ - sync_time) /
                             (back_data.time_stamp_ - front_data.time_stamp_);
        double back_scale = (sync_time - front_data.time_stamp_) /
                            (back_data.time_stamp_ - front_data.time_stamp_);

        ImuData synced_data;
        /*不插值量拷贝*/
        synced_data.time_stamp_ = sync_time;

        /*加速度插值*/
        synced_data.linear_acceleration_.x =
            front_data.linear_acceleration_.x * front_scale +
            back_data.linear_acceleration_.x * back_scale;

        synced_data.linear_acceleration_.y =
            front_data.linear_acceleration_.y * front_scale +
            back_data.linear_acceleration_.y * back_scale;

        synced_data.linear_acceleration_.z =
            front_data.linear_acceleration_.z * front_scale +
            back_data.linear_acceleration_.z * back_scale;

        /*速度插值*/
        synced_data.angular_velocity_.x =
            front_data.angular_velocity_.x * front_scale +
            back_data.angular_velocity_.x * back_scale;

        synced_data.angular_velocity_.y =
            front_data.angular_velocity_.y * front_scale +
            back_data.angular_velocity_.y * back_scale;

        synced_data.angular_velocity_.z =
            front_data.angular_velocity_.z * front_scale +
            back_data.angular_velocity_.z * back_scale;

        /*四元数插值并归一化  更准确的可以用球面插值*/
        synced_data.orientation_.x =
            front_data.orientation_.x * front_scale +
            back_data.orientation_.x * back_scale;

        synced_data.orientation_.y =
            front_data.orientation_.y * front_scale +
            back_data.orientation_.y * back_scale;

        synced_data.orientation_.z =
            front_data.orientation_.z * front_scale +
            back_data.orientation_.z * back_scale;

        synced_data.orientation_.w =
            front_data.orientation_.w * front_scale +
            back_data.orientation_.w * back_scale;

        synced_data.orientation_.Normlize();

        synced_data_buff.push_back(synced_data);

        return true;
    }
} // namespace multisensor_localization
