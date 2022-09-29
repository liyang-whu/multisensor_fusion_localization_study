#ifndef _IMU_DATA_H
#define _IMU_DATA_H

#include "../../head.hpp"

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
        //四元数转换为旋转矩阵
        Eigen::Matrix3f OrientationToRotation();
        static bool SyncData(deque<ImuData> &unsynced_data_buff,
                      deque<ImuData> &synced_data_buff,
                       double sync_time);
    };
}

#endif