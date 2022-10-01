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

    /**
     * @brief  imu坐标系下线速度、加速度转换为固连刚体lidar坐标系统下
     * @note 数学原理https://blog.csdn.net/Galaxy_Robot/article/details/108687691
     * @todo
     **/
    void VelocityData::TransformCoordinate(Eigen::Matrix4f transform_matrix)
    {
        Eigen::Matrix4d matrix = transform_matrix.cast<double>();
        /*提取旋转矩阵*/
        Eigen::Matrix3d t_R = matrix.block<3, 3>(0, 0);
        /*提取线速度、加速度*/
        Eigen::Vector3d w(angular_velocity_.x, angular_velocity_.y, angular_velocity_.z);
        Eigen::Vector3d v(linear_velocity_.x, linear_velocity_.y, linear_velocity_.z);

        /*计算转换后的角速度*/
        w = t_R * w;
        angular_velocity_.x = w(0);
        angular_velocity_.y = w(1);
        angular_velocity_.z = w(2);
        /*计算转换后的线速度 V=v + w x r*/
        v=t_R*v;
        Eigen::Vector3d r(matrix(0, 3), matrix(1, 3), matrix(2, 3));
        Eigen::Vector3d delta_v;
        delta_v(0) = w(1) * r(2) - w(2) * r(1);
        delta_v(1) = w(2) * r(0) - w(0) * r(2);
        delta_v(2) = w(0) * r(1) - w(1) * r(0);
        v=v+delta_v;
        linear_velocity_.x=v(0);
        linear_velocity_.x=v(1);
        linear_velocity_.x=v(2);
        
    }

} // namespace multisensor_localization