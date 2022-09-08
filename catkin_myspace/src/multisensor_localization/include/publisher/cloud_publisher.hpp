#ifndef _CLOUD_PUBLISHER_H
#define _CLOUD_PUBLISHER_H

#include "../../head.hpp"
#include "../sensor_data/cloud_data.hpp"

namespace multisensor_localization
{

    class CloudPublisher
    {
    public:
        CloudPublisher(ros::NodeHandle &nh, string topic_name, size_t buff_size,string frame_id);
        CloudPublisher() = default;
        void Publish(CloudData::CLOUD_PTR cloud_input_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        string frame_id_;
    };

} // namespace multisensor_localization

#endif