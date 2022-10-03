/*
 * @Description: 自定义的点云数据类型
 * @Author: Robotics gang
 * @note  modified from Ren Qian
 * @Date: 2022-10-02
 */

#ifndef CLOUD_DATA_HPP_
#define CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace multisensor_localization
{
    class CloudData
    {
    public:
        using POINT = pcl::PointXYZ;
        using CLOUD = pcl::PointCloud<POINT>;
        using CLOUD_PTR = CLOUD::Ptr;

    public:
        CloudData();

    public:
        double time_stamp_ = 0.0;
        CLOUD_PTR cloud_ptr_;
    };
} // namespace multisensor_localization

#endif