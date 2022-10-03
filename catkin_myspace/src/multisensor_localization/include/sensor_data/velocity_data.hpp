/*
 * @Description: 自定义的速度数据类型
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */

#ifndef _VELOCITY_DATA_H
#define _VELOCITY_DATA_H

#include <deque>
#include <Eigen/Dense>

namespace multisensor_localization
{

    class VelocityData
    {
    public:
        struct LinearVelocity
        {
            double x = 0.0, y = 0.0, z = 0.0;
        };
        struct AngularVelocity
        {
            double x = 0.0, y = 0.0, z = 0.0;
        };

        double time_stamp_ = 0.0;
        LinearVelocity linear_velocity_;
        AngularVelocity angular_velocity_;

    public:
        static bool SyncData(std::deque<VelocityData> &unsynced_data_buff,
                             std::deque<VelocityData> &synced_data_buff,
                            const  double sync_time);
        void TransformCoordinate(Eigen::Matrix4f transform_matrix);
    };

} // namespace multisensor_localization

#endif