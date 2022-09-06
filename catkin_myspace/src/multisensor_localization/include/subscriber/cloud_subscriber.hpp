#ifndef _CLOUD_SUBSCRIBER_H
#define _CLOUD_SUBSCRIBER_H

#include "../../head.hpp"
#include "../sensor_data/cloud_data.hpp"

namespace multisensor_localization
{
    class CloudSubscriber
    {
    public:
        CloudSubscriber(ros::NodeHandle &nh, string topic_name, size_t buff_size);
        CloudSubscriber() = default;
        void ParseData(deque<CloudData> &cloud_data_buff);

    private:
        void MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        deque<CloudData> new_cloud_data_;
    };
}

#endif