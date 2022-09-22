#ifndef _REGISTRATION_INTERFACE_H
#define _REGISTRATION_INTERFACE_H
//通用头文件
#include "../../../head.hpp"
//传感器数据类型
#include "../include/sensor_data/cloud_data.hpp"

namespace multisensor_localization
{
    class RegistrationInterface
    {
    public:
        virtual ~RegistrationInterface() = default;

        virtual bool SetTarget(const CloudData::CLOUD_PTR &input_target) = 0;
        virtual bool ScanMatch(
            const CloudData::CLOUD_PTR &cloud_in_ptr,
              const Eigen::Matrix4f &pose_in,
            CloudData::CLOUD_PTR cloud_out_ptr,
            Eigen::Matrix4f &pose_out) = 0;
    };
} // namespace name

#endif