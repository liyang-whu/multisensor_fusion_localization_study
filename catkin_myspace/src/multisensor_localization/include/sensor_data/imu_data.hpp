/*
 * @Description: 自定义的imu数据类型
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */

#ifndef IMU_DATA_HPP_
#define IMU_DATA_HPP_

#include <deque>
#include <cmath>
#include <Eigen/Dense>

namespace multisensor_localization
{
    class ImuData
    {
    public:
        struct LinearAcceleration
        {
            double x = 0.0, y = 0.0, z = 0.0;
        };
        struct AngularVelocity
        {
            double x = 0.0, y = 0.0, z = 0.0;
        };
        class Orientation
        {
        public:
            double x = 0.0, y = 0.0, z = 0.0, w = 0.0;

        public:
            void Normlize()
            {
                double nrom = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
                x /= nrom;
                y /= nrom;
                z /= nrom;
                w /= nrom;
            }
        };

        double time_stamp_ = 0.0;
        LinearAcceleration linear_acceleration_;
        AngularVelocity angular_velocity_;
        Orientation orientation_;

    public:
        Eigen::Matrix3f OrientationToRotation();
        static bool SyncData(std::deque<ImuData> &unsynced_data_buff,
                             std::deque<ImuData> &synced_data_buff,
                           const   double sync_time);
    };
}

#endif