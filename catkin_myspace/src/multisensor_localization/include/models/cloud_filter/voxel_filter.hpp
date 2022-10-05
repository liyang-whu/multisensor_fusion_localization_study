/*
 * @Description:体素滤波器
 * @Author: Robotic Gang
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#ifndef VOXEL_FILTER_HPP_
#define VOXEL_FILTER_HPP_

#include "./cloud_filter_interface.hpp"
#include <pcl/filters/voxel_grid.h>

namespace multisensor_localization
{

    class VoxelFilter : public CloudFilterInterface
    {
    public:
        VoxelFilter(const YAML::Node &node);
        VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);
        bool Filter(const CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &filtered_cloud_ptr) override;

    private:
        bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    private:
        pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
    };

} // multisensor_localization

#endif
