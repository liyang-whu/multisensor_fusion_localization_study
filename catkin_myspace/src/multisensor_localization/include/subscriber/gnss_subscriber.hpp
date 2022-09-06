#ifndef _GNSS_SUBSCRIBER_H
#define _GNSS_SUBSCRIBER_H

#include "../../head.hpp"
#include "../sensor_data/gnss_data.hpp"

namespace multisensor_localization
{
    class GnssSubscriber
    {
    public:
        GnssSubscriber(ros::NodeHandle &nh, string topic_name, size_t buff_size);
        GnssSubscriber() = default;
        void ParseData(deque<GnssData> &gnss_data_buff);

    private:
        void MsgCallback(const sensor_msgs::NavSatFixConstPtr &nav_msgs_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        deque<GnssData> new_gnss_data_;
    };
}

#endif