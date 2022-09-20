#include "../../../include/models/cloud_filter/voxel_filter.hpp"

namespace multisensor_localization
{
    VoxelFilter::VoxelFilter(const YAML::Node &node)
    {
    }
    VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z)
    {
    }

    bool VoxelFilter::Filter(const CloudData::CLOUD_PTR &input_clodu_ptr,
                             CloudData::CLOUD_PTR &filtered_cloud_ptr)
    {
    }

}
