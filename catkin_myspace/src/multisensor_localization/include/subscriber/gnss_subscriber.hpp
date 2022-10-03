#ifndef  GNSS_SUBSCRIBER_HPP_
#define GNSS_SUBSCRIBER_HPP_


#include "../sensor_data/gnss_data.hpp"
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>

namespace multisensor_localization
{
    class GnssSubscriber
    {
    public:
        GnssSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
        GnssSubscriber() = default;
        void ParseData(std::deque<GnssData> &gnss_data_buff);

    private:
        void MsgCallback(const sensor_msgs::NavSatFixConstPtr &nav_msgs_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<GnssData> new_gnss_data_;
    };
}

#endif