
#include "../../include/subscriber/gnss_subscriber.hpp"

namespace multisensor_localization
{
    /**
     * @brief  gnss初始化
     * @note
     * @todo
     **/
    GnssSubscriber::GnssSubscriber(ros::NodeHandle &nh, \
    std::string topic_name, size_t buff_size) : nh_(nh)
    {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &GnssSubscriber::MsgCallback, this);
    }

    /**
     * @brief  gnss回调函数
     * @note
     * @todo
     **/
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

    /**
     * @brief 读取gnss数据
     * @note
     * @todo
     **/
    void GnssSubscriber::ParseData(std::deque<GnssData> &gnss_data_buff)
    {
        if (new_gnss_data_.size() > 0)
        {
            gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
            new_gnss_data_.clear();
        }
    }

}