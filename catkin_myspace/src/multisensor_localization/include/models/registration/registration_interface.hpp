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
    virtual ~RegistrationInterface()=default;
        virtual bool SetTarget(const CloudData::CLOUD_PTR &target)=0;

        virtual bool ScanMatch(
            const CloudData::CLOUD_PTR &cloud_in,
            const CloudData::CLOUD_PTR cloud_out,
            const Eigen::Matrix4f &pose_predict,
            const Eigen::Matrix4f &pose_result) = 0;
    };
} // namespace name

#endif