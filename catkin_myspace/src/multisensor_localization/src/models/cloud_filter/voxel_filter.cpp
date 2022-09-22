#include "../../../include/models/cloud_filter/voxel_filter.hpp"

namespace multisensor_localization
{

    /**
     @brief VoxelFilter初始化(YAML读参)
    */
    VoxelFilter::VoxelFilter(const YAML::Node &node)
    {
        float leaf_size_x = node["leaf_size"][0].as<float>();
        float leaf_size_y = node["leaf_size"][1].as<float>();
        float leaf_size_z = node["leaf_size"][2].as<float>();

        SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
    }

    /**
     @brief VoxelFilter初始化(函数传参)
    */
    VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z)
    {
        SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
    }

    /**
      @brief voxel 滤波
     */
    bool VoxelFilter::Filter(const CloudData::CLOUD_PTR &input_cloud_ptr,
                             CloudData::CLOUD_PTR &filtered_cloud_ptr)
    {
        voxel_filter_.setInputCloud(input_cloud_ptr);
        voxel_filter_.filter(*filtered_cloud_ptr);
        return true;
    }

    /**
     @brief 设置voxel滤波参数
    */
    bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z)
    {
        voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
        LOG(INFO) << endl
                  << fontColorYellow << "滤波器参数" << fontColorReset << endl
                  << fontColorBlue
                  << leaf_size_x << " " << leaf_size_y << " " << leaf_size_z<<endl
                  << fontColorReset << endl;
        return true;
    }

}
