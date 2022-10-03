/*
 * @Description:自定义点云数据结构
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#include "../../include/sensor_data/cloud_data.hpp"

namespace multisensor_localization
{

    /**
     * @brief 点云数据类型构造函数
     * @note 为cloud_ptr_分配内存空间
     * @todo
     **/
    CloudData::CloudData() : cloud_ptr_(new CLOUD())
    {
    }

}; // namespace multisensor_localization
