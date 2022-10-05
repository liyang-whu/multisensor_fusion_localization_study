/*
 * @Description:点云匹配函数
 * @Author: Robotic Gang
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#ifndef REGISTRATION_FILTER_INTERFACE_HPP_
#define REGISTRATION_FILTER_INTERFACE_HPP_

#include "../../../include/sensor_data/cloud_data.hpp"

namespace multisensor_localization
{

    class RegistrationInterface
    {
    public:
        virtual ~RegistrationInterface() = default;
        virtual bool SetInputTarget(const CloudData::CLOUD_PTR &input_cloud) = 0;
      virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_cloud_ptr, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) =0;
    };

} // namespace multisensor_localization

#endif