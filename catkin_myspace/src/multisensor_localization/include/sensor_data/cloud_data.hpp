#ifndef _CLOUD_DATA_H
#define _CLOUD_DATA_H

#include "../../head.hpp"

namespace multisensor_localization
{
    class CloudData
    {
    public:
        using POINT = pcl::PointXYZ;
        using CLOUD = pcl::PointCloud<POINT>;
        using CLOUD_PTR = CLOUD::Ptr;

    public:
        CloudData() : cloud_ptr_(new CLOUD()) {}

    public:
        double time_stamp_ = 0.0;
        CLOUD_PTR cloud_ptr_;
    };
} // namespace multisensor_localization

#endif