#ifndef _VOXEL_FILTER_H
#define _VOXEL_FILTER_H

#include "./cloud_filter_interface.hpp"

namespace multisensor_localization
{

    class VoxelFilter : public CloudFilterInterface
    {
    public:
        VoxelFilter(const YAML::Node &node);
        VoxelFilter(float leaf_size_x,float leaf_size_y,float leaf_size_z);

          bool Filter(const CloudData::CLOUD_PTR &input_clodu_ptr,
                            CloudData::CLOUD_PTR &filtered_cloud_ptr) override;
    }

} // namespace namespace multisensor_localization

#endif