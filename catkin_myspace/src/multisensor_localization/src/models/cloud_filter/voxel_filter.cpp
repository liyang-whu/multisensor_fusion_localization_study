#include "../../../include/models/cloud_filter/voxel_filter.hpp"

namespace multisensor_localization
{
   /**
     * @brief VoxelFilter初始化(参数节点方式)
     * @note 当前node为local_map、frame、display
     * @todo
     **/
    VoxelFilter::VoxelFilter(const YAML::Node &node)
    {
        float leaf_size_x = node["leaf_size"][0].as<float>();
        float leaf_size_y = node["leaf_size"][1].as<float>();
        float leaf_size_z = node["leaf_size"][2].as<float>();

             LOG(INFO) << endl
                  << fontColorYellow <<"滤波器尺寸" << fontColorReset << endl
                  << fontColorBlue
                  << leaf_size_x << " " << leaf_size_y << " " << leaf_size_z<<endl
                  << fontColorReset << endl;

        SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
    }

      /**
     * @brief VoxelFilter滤波器初始化(函数传参方式)
     * @note 
     * @todo
     **/
    VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z)
    {
        SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
    }


     /**
     * @brief 设定Voxel滤波尺寸
     * @note 
     * @todo
     **/
    bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z)
    {
        voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
        return true;
    }

       /**
     * @brief 滤波
     * @note 
     * @todo
     **/
    bool VoxelFilter::Filter(const CloudData::CLOUD_PTR &input_cloud_ptr,
                             CloudData::CLOUD_PTR &filtered_cloud_ptr)
    {
        voxel_filter_.setInputCloud(input_cloud_ptr);
        voxel_filter_.filter(*filtered_cloud_ptr);
        return true;
    }

}
