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
        struct Orientation
        {
            double x = 0.0, y = 0.0, z = 0.0, w = 0.0;
        };

        double time_stamp_ = 0.0;
        LinearAcceleration linear_acceleration_;
        AngularVelocity angular_velocity_;
        Orientation orientation_;

    public:
        //四元数转换为旋转矩阵
        Eigen::Matrix3f OrientationToRotation()
        {
            Eigen::Quaterniond q(orientation_.w, orientation_.x, orientation_.y, orientation_.z);
            Eigen::Matrix3f  rotation=q.matrix().cast<float>();
            return rotation;
        }
    };
}

#endif