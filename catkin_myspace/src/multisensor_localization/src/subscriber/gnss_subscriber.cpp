#include "../../head.hpp"
#include "../../include/subscriber/gnss_subscriber.hpp"

namespace multisensor_localization
{
    // gnss初始化订阅
    GnssSubscriber::GnssSubscriber(ros::NodeHandle &nh, string topic_name, size_t buff_size) : nh_(nh)
    {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &GnssSubscriber::MsgCallback, this);
    }

    // gnss回调函数
    void GnssSubscriber::MsgCallback(const sensor_msgs::NavSatFixConstPtr &nav_msgs_ptr)
    {
        GnssData gnss_data;
        gnss_data.time_stamp_ = nav_msgs_ptr->header.stamp.toSec();
        gnss_data.latitude_ = nav_msgs_ptr->latitude;
        gnss_data.longtitude_ = nav_msgs_ptr->longitude;
        gnss_data.altitude_ = nav_msgs_ptr->status.status;
        gnss_data.service_ = nav_msgs_ptr->status.service;

        new_gnss_data_.push_back(gnss_data);
    }

    //读取gnss数据
    void GnssSubscriber::ParseData(deque<GnssData> &gnss_data_buff)
    {
        if (new_gnss_data_.size() > 0)
        {
            gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
            new_gnss_data_.clear();
        }
    }

}