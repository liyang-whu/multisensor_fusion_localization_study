/*
 * @Description:gnss订阅
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#include "../../include/subscriber/gnss_subscriber.hpp"

namespace multisensor_localization
{

    /**
     * @brief GNSS订阅初始化
     * @note
     * @todo
     **/
    GnssSubscriber::GnssSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
        : nh_(nh)
    {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &GnssSubscriber::MsgCallback, this);
    }

    /**
     * @brief GNSS回调函数
     * @note
     * @todo
     **/
    void GnssSubscriber::MsgCallback(const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr)
    {
        GnssData gnss_data;
        gnss_data.time_stamp_ = nav_sat_fix_ptr->header.stamp.toSec();
        gnss_data.latitude_ = nav_sat_fix_ptr->latitude;
        gnss_data.longitude_ = nav_sat_fix_ptr->longitude;
        gnss_data.altitude_ = nav_sat_fix_ptr->altitude;
        gnss_data.status_ = nav_sat_fix_ptr->status.status;
        gnss_data.service_ = nav_sat_fix_ptr->status.service;

        new_gnss_data_buff_.push_back(gnss_data);
    }

    /**
     * @brief 读取缓冲区并清空
     * @note
     * @todo
     **/
    void GnssSubscriber::ParseData(std::deque<GnssData> &gnss_data_buff)
    {
        if (new_gnss_data_buff_.size() > 0)
        {
            gnss_data_buff.insert(gnss_data_buff.end(),
                                  new_gnss_data_buff_.begin(), new_gnss_data_buff_.end());
            new_gnss_data_buff_.clear();
        }
    }
}