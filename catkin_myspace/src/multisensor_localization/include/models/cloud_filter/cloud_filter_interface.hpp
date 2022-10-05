/*
 * @Description:点云滤波接口函数
 * @Author: Robotic Gang
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#ifndef CLOUD_FILTER_INTERFACE_HPP_
#define CLOUD_FILTER_INTERFACE_HPP_

#include "../../../include/sensor_data/cloud_data.hpp"
#include <yaml-cpp/yaml.h>

namespace multisensor_localization
{
    class CloudFilterInterface
    {
    public:
        virtual ~CloudFilterInterface() = default;
        virtual bool Filter(const CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &filtered_cloud_ptr) = 0;
    };

} // namespace multisensor_localization

#endif