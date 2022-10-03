/*
 * @Description: 自定义的点云数据类型
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */

#include "../../include/sensor_data/cloud_data.hpp"

namespace multisensor_localization
{
    /**
     * @brief  为cloud_ptr_分配内存空间
     * @note
     * @todo
     **/
    CloudData::CloudData() : cloud_ptr_(new CLOUD())
    {
    }
    
} // namespace multisensor_localization