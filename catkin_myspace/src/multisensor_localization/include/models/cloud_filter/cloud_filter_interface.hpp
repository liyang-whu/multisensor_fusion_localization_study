#ifndef _CLOUD_FILTER_INTERFACE_H
#define _CLOUD_FILTER_INTERFACE_H

#include "../../../head.hpp"
#include "../../sensor_data/cloud_data.hpp"

namespace multisensor_localization
{

    class CloudFilterInterface
    {
    public:
        virtual ~CloudFilterInterface() = default;
        virtual bool Filter(const CloudData::CLOUD_PTR &input_clodu_ptr,
                            CloudData::CLOUD_PTR &filtered_cloud_ptr) = 0;
    };

} // namespace multisensor_localization

#endif